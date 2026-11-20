"""JHB material/cap detection, pairing and Lab color classification."""

from dataclasses import dataclass
import json
from pathlib import Path

import cv2
import numpy as np

from .material_model import MaterialModelError
from .ring_geometry import GeometryConfig, RingGeometry


@dataclass(frozen=True)
class JhbDetection:
    label: str
    confidence: float
    bbox_xyxy: tuple


class JhbMaterialModel:
    def __init__(self, config, *, load_color=True):
        self.p = config
        path = Path(config['jhb.model_path']).expanduser()
        if not path.is_file():
            raise ValueError(f'模型文件不存在：{path}')
        if path.suffix == '.onnx':
            self.net = cv2.dnn.readNetFromONNX(str(path))
            self.bpu = None
        else:
            from hobot_dnn import pyeasy_dnn
            self.bpu = pyeasy_dnn.load(str(path))[0]
            self.net = None
            shapes = [tuple(output.properties.shape) for output in self.bpu.outputs]
            expected = [(1, 384 // s, 640 // s, c) for s in (8, 16, 32) for c in (2, 64)]
            if sorted(shapes) != sorted(expected):
                raise ValueError(f'JHB 模型输出形状不匹配：{shapes}')
        self.geometry = RingGeometry(GeometryConfig(**{
            k.removeprefix('jhb.geometry.'): v for k, v in config.items()
            if k.startswith('jhb.geometry.')}), contour_boundary='outer')
        if not load_color:
            return
        data = json.loads(Path(config['jhb.color_path']).expanduser().read_text())
        if data.get('format') != 'gc_color_lab' or data.get('version') != 1:
            raise ValueError('需要 gc_color_lab v1 判色模型')
        core = data['model']['core']
        self.mu = np.asarray(core['mu'], dtype=float)
        self.icov = np.asarray(core['icov'], dtype=float)
        self.logdet = np.asarray(core['logdet'], dtype=float)
        if (self.mu.shape != (6, 3) or self.icov.shape != (6, 3, 3)
                or self.logdet.shape != (6,) or not all(
                    np.isfinite(a).all() for a in (self.mu, self.icov, self.logdet))):
            raise ValueError('Lab 模型须包含六组有限的 mu/icov/logdet')
        self.region = data['info']['region']
        if (not 0 < self.region['core_ratio'] <= 1 or type(self.region['stride']) is not int
                or self.region['stride'] < 1 or type(self.region['fine_scale']) is not int
                or self.region['fine_scale'] < 1):
            raise ValueError('Lab 取样参数无效')
        self.white_l = data['model']['white_L']
        self.white_c = data['model']['white_C']

    def infer(self, image_bgr):
        try:
            p = self.p
            height, width = image_bgr.shape[:2]
            divisor = p['jhb.work_scale']
            work = cv2.resize(image_bgr, (max(1, width // divisor), max(1, height // divisor)),
                              interpolation=cv2.INTER_AREA)
            h, w = work.shape[:2]
            ratio = min(640 / w, 384 / h)
            rw, rh = min(640, max(1, round(w * ratio))), min(384, max(1, round(h * ratio)))
            left, top = (640 - rw) // 2, (384 - rh) // 2
            boxed = cv2.copyMakeBorder(cv2.resize(work, (rw, rh)), top, 384-rh-top,
                                       left, 640-rw-left, cv2.BORDER_CONSTANT, value=(114,114,114))
            if self.net is not None:
                self.net.setInput(cv2.dnn.blobFromImage(boxed, 1/255, swapRB=True))
                outs = self.net.forward(['cls_s8','box_s8','cls_s16','box_s16','cls_s32','box_s32'])
            else:
                # OpenCV I420 uses limited range; this model was trained with JPEG full range.
                yuv = cv2.cvtColor(boxed, cv2.COLOR_BGR2YCrCb)
                nv12 = np.empty((384 * 3 // 2, 640), np.uint8)
                nv12[:384] = yuv[:, :, 0]
                nv12[384:, 0::2] = yuv[::2, ::2, 2]
                nv12[384:, 1::2] = yuv[::2, ::2, 1]
                tensors = self.bpu.forward(nv12)
                arrays = [np.asarray(t.buffer) for t in tensors]
                if any(a.dtype != np.float32 for a in arrays):
                    raise ValueError('请使用带 float32 反量化输出的 713700d9dc72.bin，或 ONNX')
                outs = [next(a for a in arrays if a.shape == (1,384//s,640//s,c))
                        for s in (8,16,32) for c in (2,64)]
            boxes, scores, classes = decode(outs, p['jhb.confidence_threshold'])
            result = []
            for i in class_nms(boxes, scores, classes, p['jhb.nms_threshold']):
                box = boxes[i].copy()
                box[[0,2]] = np.clip((box[[0,2]]-left) / (rw/w), 0, w) * (width/w)
                box[[1,3]] = np.clip((box[[1,3]]-top) / (rh/h), 0, h) * (height/h)
                if box[2] > box[0] and box[3] > box[1]:
                    result.append(JhbDetection(('material','cap_top')[classes[i]],
                                               float(scores[i]), tuple(float(v) for v in box)))
            return result
        except Exception as error:
            raise MaterialModelError('MODEL_INFERENCE_FAILED', str(error)) from error

    def pair_caps(self, detections, image_shape):
        p = self.p
        h, w = image_shape[:2]
        scale = np.array([w/max(1,w//p['jhb.work_scale']), h/max(1,h//p['jhb.work_scale'])])
        materials = [d for d in detections if d.label == 'material'
                     and d.confidence >= p['jhb.material_min_score']]
        pairs, used, rejected = [], set(), []
        for cap in sorted((d for d in detections if d.label == 'cap_top'),
                          key=lambda d: d.confidence, reverse=True):
            x1,y1,x2,y2 = cap.bbox_xyxy
            bw,bh = (x2-x1)/scale[0], (y2-y1)/scale[1]
            border = min(x1/scale[0],y1/scale[1],(w-x2)/scale[0],(h-y2)/scale[1])
            reason = None
            if cap.confidence < p['jhb.cap_min_score']:
                reason = 'CAP_LOW_SCORE'
            elif border < p['jhb.border_px']:
                reason = 'TRUNCATED'
            elif min(bw,bh)/max(bw,bh) < p['jhb.min_axis_ratio']:
                reason = 'CAP_AXIS_RATIO'
            candidates = []
            if reason is None:
                for i, material in enumerate(materials):
                    a,b,c,d = material.bbox_xyxy
                    overlap = max(0,min(x2,c)-max(x1,a))*max(0,min(y2,d)-max(y1,b)) / ((x2-x1)*(y2-y1))
                    if i not in used and overlap >= p['jhb.min_containment']:
                        candidates.append((overlap,material.confidence,i))
                if candidates:
                    index = max(candidates)[2]
                    used.add(index)
                    pairs.append((cap, materials[index]))
                else:
                    reason = 'CAP_UNPAIRED'
            if reason:
                rejected.append({'bbox_xyxy': cap.bbox_xyxy, 'reason': reason})
        return pairs, rejected

    def color_context(self, image, detections):
        p = self.p
        h,w = image.shape[:2]
        fine_scale = self.region['fine_scale']
        fine = cv2.resize(image, (max(1,w//fine_scale),max(1,h//fine_scale)), interpolation=cv2.INTER_AREA)
        gains = np.ones(3)
        warnings = []
        if p['jhb.white_balance']:
            work = cv2.resize(image, (max(1,w//p['jhb.work_scale']),max(1,h//p['jhb.work_scale'])), interpolation=cv2.INTER_AREA)
            wh,ww = work.shape[:2]
            lab = cv2.cvtColor(work.astype(np.float32)/255, cv2.COLOR_BGR2Lab)
            neutral = (lab[:,:,0] >= self.white_l) & (np.hypot(lab[:,:,1],lab[:,:,2]) <= self.white_c)
            for item in detections:
                if item.label == 'material':
                    x1,y1,x2,y2 = item.bbox_xyxy
                    neutral[max(0,int(y1*wh/h)):min(wh,int(np.ceil(y2*wh/h))),
                            max(0,int(x1*ww/w)):min(ww,int(np.ceil(x2*ww/w)))] = False
            samples = work[neutral]
            if len(samples) >= 32:
                median = np.median(samples, axis=0)
                gains = median.mean()/np.maximum(median,1)
            else:
                warnings.append('JHB_WB_NO_REFERENCE: 未找到足够亮的中性背景，使用单位白平衡增益')
        return fine, gains, warnings

    def classify(self, fine, original_shape, center, radius, gains):
        h,w = original_shape[:2]
        fh,fw = fine.shape[:2]
        cx,cy = center[0]*fw/w, center[1]*fh/h
        rx,ry = radius*self.region['core_ratio']*fw/w, radius*self.region['core_ratio']*fh/h
        stride = self.region['stride']
        xs = np.arange(max(0,int(np.floor(cx-rx))), min(fw,int(np.ceil(cx+rx))+1), stride)
        ys = np.arange(max(0,int(np.floor(cy-ry))), min(fh,int(np.ceil(cy+ry))+1), stride)
        xx,yy = np.meshgrid(xs,ys)
        mask = ((xx-cx)/max(rx,1e-6))**2 + ((yy-cy)/max(ry,1e-6))**2 <= 1
        samples = fine[yy[mask],xx[mask]]
        if not len(samples):
            return 0, 0.0, None, 'COLOR_NO_PIXELS'
        bgr = np.clip(np.median(samples.astype(float)*gains,axis=0),0,255).astype(np.float32)/255
        lab = cv2.cvtColor(bgr.reshape(1,1,3),cv2.COLOR_BGR2Lab)[0,0]
        delta = lab-self.mu
        d2 = np.einsum('ni,nij,nj->n',delta,self.icov,delta)
        costs = d2+self.logdet
        best = int(costs.argmin())
        posterior = np.exp(-(costs-costs.min())/2)
        probability = float(posterior[best]/posterior.sum())
        reason = ('COLOR_LOW_CONFIDENCE' if probability < self.p['jhb.color_p_min'] else
                  'COLOR_OUT_OF_DISTRIBUTION' if d2[best] > self.p['jhb.color_d2_max'] else 'OK')
        return best+1 if reason == 'OK' else 0, probability, float(d2[best]), reason


def decode(outs, threshold):
    boxes,scores,classes = [],[],[]
    expected = [(1,384//s,640//s,c) for s in (8,16,32) for c in (2,64)]
    if len(outs) != 6 or any(a.shape != shape or not np.isfinite(a).all() for a,shape in zip(outs,expected)):
        raise ValueError('JHB 输出须为六个有限 NHWC 分类/DFL 头')
    for level,stride in enumerate((8,16,32)):
        logits = outs[level*2].reshape(-1,2)
        prob = 1/(1+np.exp(-np.clip(logits,-80,80)))
        best = prob.max(axis=1)
        rows = np.flatnonzero(best >= threshold)
        reg = outs[level*2+1].reshape(-1,4,16)[rows]
        e = np.exp(reg-reg.max(axis=2,keepdims=True))
        dist = (e*np.arange(16,dtype=np.float32)).sum(axis=2)/e.sum(axis=2)
        ctr = np.stack((rows%(640//stride)+.5,rows//(640//stride)+.5),axis=1)
        boxes.append(np.concatenate((ctr-dist[:,:2],ctr+dist[:,2:]),axis=1)*stride)
        scores.append(best[rows])
        classes.append(prob[rows].argmax(axis=1))
    return np.concatenate(boxes),np.concatenate(scores),np.concatenate(classes)


def class_nms(boxes, scores, classes, threshold):
    order = np.argsort(-scores,kind='stable')
    area = np.maximum(0,boxes[:,2]-boxes[:,0])*np.maximum(0,boxes[:,3]-boxes[:,1])
    keep = []
    while len(order):
        i,rest = int(order[0]),order[1:]
        keep.append(i)
        wh = np.maximum(0,np.minimum(boxes[i,2:],boxes[rest,2:])-np.maximum(boxes[i,:2],boxes[rest,:2]))
        inter = wh[:,0]*wh[:,1]
        union = area[i]+area[rest]-inter
        iou = np.divide(inter,union,out=np.zeros_like(inter),where=union>0)
        order = rest[(classes[rest] != classes[i]) | (iou <= threshold)]
    return keep
