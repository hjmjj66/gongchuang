"""Reusable material detection without camera, ROS or HTTP dependencies."""

from dataclasses import asdict, dataclass, field
import math
import time

import cv2
import numpy as np

from .material_config import COLORS, parse_material_config
from .material_geometry import TopCircleLocator
from .material_hsv import detect_hsv
from .material_model import MaterialModelDetection, MaterialModelError


@dataclass
class MaterialObservation:
    color_id: int
    color_name: str
    raw_label: str
    identity_source: str
    bbox_xyxy: tuple
    bbox_center_px: tuple
    valid: bool = False  # Both localization and color identity are usable.
    status: str = 'GEOMETRY_FAILED'
    center_px: tuple = None
    center_source: str = None
    circle_center_px: tuple = None
    radius_px: float = None
    yolo_confidence: float = None
    roi_xyxy: tuple = None
    geometry_reason: str = None
    circle_support: float | dict = None
    contour_area_px2: float = None
    conflicting_colors: list = field(default_factory=list)
    color_overlap_ratio: float = None
    material_bbox_xyxy: tuple = None
    material_confidence: float = None
    color_confidence: float = None
    color_bbox_xyxy: tuple = None
    color_d2: float = None
    color_reason: str = None
    radii_px: tuple = ()
    ellipses_px: tuple = ()


@dataclass
class MaterialFrameResult:
    frame_seq: int
    stamp: float
    image_size: tuple
    backend: str
    model_kind: str
    search_roi_xyxy: tuple
    status: str = 'NO_DETECTION'
    processing_ms: float = 0.0
    stages_ms: dict = field(default_factory=dict)
    observations: list = field(default_factory=list)
    warnings: list = field(default_factory=list)
    rejected_candidates: list = field(default_factory=list)
    error: str = None
    debug: dict = field(default_factory=dict, repr=False)

    def to_dict(self):
        # Do not deep-copy image arrays merely to throw them away afterwards.
        return {name: ([asdict(o) for o in self.observations] if name == 'observations'
                       else getattr(self, name))
                for name in self.__dataclass_fields__ if name != 'debug'}


class MaterialDetector:
    def __init__(self, config=None, model=None):
        self.config = parse_material_config({}) if config is None else config
        self.model = model
        self.geometry = TopCircleLocator(self.config)

    def detect(self, frame_bgr, frame_seq=0, stamp=0.0, target_color_id=None, search_roi=None,
               *, geometry_target=0, largest_per_color=False):
        started = time.perf_counter()
        _validate_frame(frame_bgr, frame_seq, stamp)
        p = self.config
        target = _target(p['target_color_id'] if target_color_id is None else target_color_id)
        geometry_target = _target(geometry_target)
        roi = _search_roi(frame_bgr, p['search_roi'] if search_roi is None else search_roi)
        x1,y1,x2,y2 = roi
        work = frame_bgr[y1:y2,x1:x2]
        result = self._new_result(frame_bgr, frame_seq, stamp, roi)
        stage = time.perf_counter()
        if p['test_combine']:
            if self.model is None:
                result.status, result.error = 'MODEL_NOT_CONFIGURED', '未加载 test_combine 双模型'
                return _finish(result, started, preserve_status=True)
            try:
                detections, colors, stages = self.model.infer(work)
                result.stages_ms.update(stages)
                self._combine_test(work, detections, colors, roi, target, result, geometry_target)
            except (MaterialModelError, ValueError, RuntimeError, cv2.error) as error:
                result.status = getattr(error, 'status', 'MODEL_INFERENCE_FAILED')
                result.error = str(error)
                result.observations.clear()
                return _finish(result, started, preserve_status=True)
        elif p['use_jhb_model']:
            if self.model is None:
                result.status, result.error = 'MODEL_NOT_CONFIGURED', '未加载 JHB 物料模型'
                return _finish(result, started, preserve_status=True)
            try:
                detections = self.model.infer(work)
                result.stages_ms['yolo'] = (time.perf_counter()-stage)*1000
                self._combine_jhb(work, detections, roi, target, result, geometry_target)
            except (MaterialModelError, ValueError, RuntimeError, cv2.error) as error:
                result.status = getattr(error, 'status', 'MODEL_INFERENCE_FAILED')
                result.error = str(error)
                result.observations.clear()
                return _finish(result, started, preserve_status=True)
        elif p['material_backend'] == 1:
            candidates, debug = detect_hsv(work, p)
            result.stages_ms['hsv'] = (time.perf_counter()-stage)*1000
            result.debug.update(debug)
            if debug['missing_colors']:
                result.warnings.append(f"HSV_UNCONFIGURED_COLORS: {debug['missing_colors']}")
            for candidate in candidates:
                if target and candidate['color_id'] != target:
                    continue
                box = _offset_box(candidate['bbox_xyxy'], x1, y1)
                item = self._observation(candidate['color_id'], candidate['color_name'], box, 'hsv')
                item.contour_area_px2 = candidate['contour_area_px2']
                item.conflicting_colors = sorted(set(candidate['conflicting_colors']))
                item.color_overlap_ratio = candidate['color_overlap_ratio']
                item.geometry_reason = 'NOT_APPLICABLE'
                if item.conflicting_colors:
                    item.status = 'COLOR_AMBIGUOUS'
                elif _truncated(box, roi, p['border_tolerance_px']):
                    item.status = 'TRUNCATED'
                else:
                    item.valid, item.status = True, 'OK'
                    item.center_px = (candidate['centroid'][0]+x1, candidate['centroid'][1]+y1)
                    item.center_source = 'contour_centroid'
                result.observations.append(item)
            result.debug['contours'] = [c['contour'] + (x1,y1) for c in candidates]
        else:
            if self.model is None:
                result.status, result.error = 'MODEL_NOT_CONFIGURED', '未加载物料模型'
                return _finish(result, started, preserve_status=True)
            try:
                boxes = self.model.infer(work)
            except MaterialModelError as error:
                result.status, result.error = error.status, str(error)
                return _finish(result, started, preserve_status=True)
            result.stages_ms['yolo'] = (time.perf_counter()-stage)*1000
            boxes = _validate_detections(boxes)
            stage = time.perf_counter()
            self._combine(frame_bgr, boxes, roi, target, result, (x1,y1), geometry_target)
            if p['material_model'] == 1 and p['top.refine_circle']:
                result.stages_ms['geometry'] = (time.perf_counter()-stage)*1000
        return _finish(result, started, largest_per_color=largest_per_color)

    def detect_boxes(self, frame_bgr, detections, frame_seq=0, stamp=0.0, target_color_id=None,
                     *, geometry_target=0, largest_per_color=False):
        started = time.perf_counter()
        _validate_frame(frame_bgr, frame_seq, stamp)
        if self.config['use_jhb_model'] or self.config['test_combine']:
            raise ValueError('JHB 需要 material/cap_top 配对，请使用 detect；detect_boxes 仅接收六色模型结果')
        if self.config['material_backend'] != 2:
            raise ValueError('detect_boxes is only available for YOLO backend')
        boxes = _validate_detections(detections)
        target = _target(self.config['target_color_id'] if target_color_id is None else target_color_id)
        geometry_target = _target(geometry_target)
        roi = (0,0,frame_bgr.shape[1],frame_bgr.shape[0])
        result = self._new_result(frame_bgr, frame_seq, stamp, roi)
        self._combine(frame_bgr, boxes, roi, target, result, geometry_target=geometry_target)
        if self.config['material_model'] == 1 and self.config['top.refine_circle']:
            result.stages_ms['geometry'] = (time.perf_counter()-started)*1000
        return _finish(result, started, largest_per_color=largest_per_color)

    def detect_roi(self, frame_bgr, bbox_xyxy, manual_color_id, frame_seq=0, stamp=0.0):
        started = time.perf_counter()
        _validate_frame(frame_bgr, frame_seq, stamp)
        color = _target(manual_color_id)
        if not color:
            raise ValueError('manual_color_id must be 1～6')
        box = MaterialModelDetection(color, COLORS[color-1], 1.0, tuple(bbox_xyxy))
        roi = (0,0,frame_bgr.shape[1],frame_bgr.shape[0])
        result = self._new_result(frame_bgr, frame_seq, stamp, roi)
        result.backend, result.model_kind = 'manual', 'upper_top'
        item = self._observation(color, box.raw_label, box.bbox_xyxy, 'manual')
        self._locate_top(frame_bgr, item, roi, result)
        result.observations.append(item)
        if self.config['top.refine_circle']:
            result.stages_ms['geometry'] = (time.perf_counter()-started)*1000
        return _finish(result, started)

    def _new_result(self, frame, sequence, stamp, roi):
        if self.config['test_combine']:
            return MaterialFrameResult(sequence, float(stamp), (frame.shape[1],frame.shape[0]),
                                       'test_combine', 'material_cap', roi)
        if self.config['use_jhb_model']:
            return MaterialFrameResult(sequence, float(stamp), (frame.shape[1],frame.shape[0]),
                                       'jhb', 'material_cap', roi)
        yolo = self.config['material_backend'] == 2
        return MaterialFrameResult(sequence, float(stamp), (frame.shape[1],frame.shape[0]),
                                   'yolo' if yolo else 'hsv',
                                   ('upper_top' if self.config['material_model'] == 1 else 'whole')
                                   if yolo else None, roi)

    @staticmethod
    def _observation(color, label, box, identity):
        x1,y1,x2,y2 = box
        return MaterialObservation(color, COLORS[color-1] if color else 'unknown', label, identity,
                                   tuple(box), ((x1+x2)/2,(y1+y2)/2))

    def _combine_jhb(self, work, detections, roi, target, result, geometry_target):
        p, model = self.config, self.model
        stage = time.perf_counter()
        pairs, rejected = model.pair_caps(detections, work.shape)
        result.rejected_candidates = [{'bbox_xyxy': _offset_box(r['bbox_xyxy'],*roi[:2]),
                                       'reason': r['reason']} for r in rejected]
        result.stages_ms['pairing'] = (time.perf_counter()-stage)*1000
        if not pairs:
            result.stages_ms.update(geometry=0., color=0.)
            return
        stage = time.perf_counter()
        fine, gains, warnings = model.color_context(work, detections)
        result.warnings.extend(warnings)
        result.debug['jhb_white_balance_gains'] = gains.tolist()
        geometry_ms, color_ms = 0.0, (time.perf_counter()-stage)*1000
        for cap, material in pairs:
            x1,y1,x2,y2 = cap.bbox_xyxy
            box_center = ((x1+x2)/2,(y1+y2)/2)
            radius = min(x2-x1,y2-y1)/2
            # Classify provisionally to avoid refining other colors in a selected-color phase.
            color, probability, distance, reason = 0,0.0,None,None
            if geometry_target:
                color_started = time.perf_counter()
                color, probability, distance, reason = model.classify(fine,work.shape,box_center,radius,gains)
                color_ms += (time.perf_counter()-color_started)*1000
            refine = p['top.refine_circle'] and (not geometry_target or color in (0,geometry_target))
            geo_started = time.perf_counter()
            item, center, radius = self._locate_jhb(work, cap, material, roi, result, refine)
            geometry_ms += (time.perf_counter()-geo_started)*1000
            color_started = time.perf_counter()
            color,probability,distance,reason = model.classify(fine,work.shape,center,radius,gains)
            color_ms += (time.perf_counter()-color_started)*1000
            item.color_id, item.color_name = color,COLORS[color-1] if color else 'unknown'
            item.color_confidence,item.color_d2,item.color_reason = probability,distance,reason
            if not color:
                # Unknown identity still permits the raw area's initial position-only follow.
                item.valid,item.status = False,'COLOR_REJECTED'
            if target and color and color != target:
                continue
            result.observations.append(item)
        result.stages_ms.update(geometry=geometry_ms,color=color_ms)

    def _locate_jhb(self, work, cap, material, roi, result, refine):
        p = self.config
        ox,oy = roi[:2]
        item = self._observation(0, 'cap_top', _offset_box(cap.bbox_xyxy,ox,oy), 'jhb_lab')
        item.yolo_confidence = cap.confidence
        item.material_bbox_xyxy = _offset_box(material.bbox_xyxy,ox,oy)
        item.material_confidence = material.confidence
        x1,y1,x2,y2 = cap.bbox_xyxy
        box_center = ((x1+x2)/2,(y1+y2)/2)
        radius = min(x2-x1,y2-y1)/2
        circle = None
        center = box_center
        if refine:
            pad = p['top.roi_padding_ratio']
            left,top = max(0,math.floor(x1-(x2-x1)*pad)),max(0,math.floor(y1-(y2-y1)*pad))
            right,bottom = min(work.shape[1],math.ceil(x2+(x2-x1)*pad)),min(work.shape[0],math.ceil(y2+(y2-y1)*pad))
            item.roi_xyxy = (left+ox,top+oy,right+ox,bottom+oy)
            item.geometry_reason = 'ROI_TOO_SMALL'
            if min(right-left,bottom-top) >= p['top.minimum_roi_size_px']:
                circle = self.model.geometry.detect(work[top:bottom,left:right],
                                                   (x1-left,y1-top,x2-left,y2-top))
                item.geometry_reason = circle.reason
                result.debug.setdefault('geometry', []).append({
                    'roi_xyxy': item.roi_xyxy, 'gray': circle.processed, 'edges': circle.edges,
                    'candidates': [{'center': (c.x,c.y), 'radius': c.radius,
                                    'reason': None if c.accepted else c.reason}
                                   for c in circle.candidates]})
            if circle is not None and circle.center_px is not None:
                center = (circle.center_px[0]+left,circle.center_px[1]+top)
                item.circle_center_px = (center[0]+ox,center[1]+oy)
                item.radii_px, item.circle_support = circle.radii_px,circle.support
                item.radius_px = max(circle.radii_px)
                item.ellipses_px = tuple(((c.x+left+ox,c.y+top+oy),
                                         (2*c.ellipse_axes[0],2*c.ellipse_axes[1]),c.ellipse_angle)
                                        for c in circle.selected if c.ellipse_axes is not None)
                item.valid, item.status = True,'OK'
                item.center_px, item.center_source = item.circle_center_px,'circle'
                radius = item.radius_px
            elif p['top.allow_bbox_fallback']:
                _use_bbox(item,item.geometry_reason)
        else:
            _use_bbox(item,'NOT_TARGET' if p['top.refine_circle'] else 'DISABLED')
        return item, center, radius

    def _combine_test(self, work, detections, colors, roi, target, result, geometry_target):
        from .material_combine import match_colors

        p = self.config
        stage = time.perf_counter()
        pairs, rejected = self.model.position.pair_caps(detections, work.shape)
        result.rejected_candidates = [{'bbox_xyxy': _offset_box(r['bbox_xyxy'], *roi[:2]),
                                       'reason': r['reason']} for r in rejected]
        result.stages_ms['pairing'] = (time.perf_counter()-stage)*1000
        stage = time.perf_counter()
        matches = match_colors(pairs, colors, p)
        result.stages_ms['matching'] = (time.perf_counter()-stage)*1000
        geometry_ms = 0.
        for (cap, material), (color, reason) in zip(pairs, matches):
            if target and color is not None and color.color_id != target:
                continue
            refine = p['top.refine_circle'] and (not geometry_target or color is None or
                                                 color.color_id == geometry_target)
            stage = time.perf_counter()
            item, _, _ = self._locate_jhb(work, cap, material, roi, result, refine)
            geometry_ms += (time.perf_counter()-stage)*1000
            item.identity_source, item.color_reason = 'yolo_jhb', reason
            if color is None:
                item.valid, item.status = False, reason
            else:
                item.color_id, item.color_name = color.color_id, COLORS[color.color_id-1]
                item.raw_label, item.color_confidence = color.raw_label, color.confidence
                item.color_bbox_xyxy = _offset_box(color.bbox_xyxy, *roi[:2])
            result.observations.append(item)
        result.stages_ms['geometry'] = geometry_ms

    def _combine(self, frame, boxes, roi, target, result, offset=(0,0), geometry_target=0):
        p = self.config
        for b in boxes:
            if b.confidence <= p['confidence_threshold'] or (target and b.color_id != target):
                continue
            item = self._observation(b.color_id, b.raw_label,
                                     _offset_box(b.bbox_xyxy, *offset), 'yolo')
            item.yolo_confidence = b.confidence
            if p['material_model'] == 1 and geometry_target and b.color_id != geometry_target:
                if self._box_valid(item, roi):
                    _use_bbox(item, 'NOT_TARGET')
            elif p['material_model'] == 1:
                self._locate_top(frame, item, roi, result)
            elif self._box_valid(item, roi):
                _use_bbox(item, 'NOT_APPLICABLE')
            result.observations.append(item)

    def _box_valid(self, item, roi):
        x1,y1,x2,y2 = item.bbox_xyxy
        left,top,right,bottom = roi
        if x2 <= left or y2 <= top or x1 >= right or y1 >= bottom:
            item.status, item.geometry_reason = 'BOX_OUTSIDE_IMAGE', 'BOX_OUTSIDE_IMAGE'
            return False
        return True

    def _locate_top(self, frame, item, roi, result):
        if not self._box_valid(item, roi):
            return
        p = self.config
        if not p['top.refine_circle']:
            _use_bbox(item, 'DISABLED')
            return
        x1,y1,x2,y2 = item.bbox_xyxy
        padding = p['top.roi_padding_ratio']
        left = max(roi[0], math.floor(x1-(x2-x1)*padding))
        top = max(roi[1], math.floor(y1-(y2-y1)*padding))
        right = min(roi[2], math.ceil(x2+(x2-x1)*padding))
        bottom = min(roi[3], math.ceil(y2+(y2-y1)*padding))
        item.roi_xyxy = (left,top,right,bottom)
        if min(right-left,bottom-top) < p['top.minimum_roi_size_px']:
            item.geometry_reason = 'ROI_TOO_SMALL'
        else:
            circle = self.geometry.detect(frame[top:bottom,left:right],
                                          (x1-left,y1-top,x2-left,y2-top))
            item.geometry_reason = circle.reason
            result.debug.setdefault('geometry', []).append({'roi_xyxy': item.roi_xyxy, **circle.debug})
            if circle.center is not None:
                item.circle_center_px = (circle.center[0]+left,circle.center[1]+top)
                item.radius_px, item.circle_support = circle.radius, circle.support
                item.valid, item.status = True, 'OK'
                item.center_px, item.center_source = item.circle_center_px, 'circle'
                return
        if p['top.allow_bbox_fallback']:
            _use_bbox(item, item.geometry_reason)
        else:
            item.status = 'GEOMETRY_FAILED'


def _finish(result, started, preserve_status=False, *, largest_per_color=False):
    result.observations.sort(key=lambda o: (o.color_id, o.bbox_xyxy[0], o.bbox_xyxy[1]))
    if largest_per_color:
        selected = {}
        for item in result.observations:
            if not item.color_id:
                continue  # Unknown identities remain available for initial position-only follow.
            x1,y1,x2,y2 = item.bbox_xyxy
            area = (x2-x1)*(y2-y1)
            previous = selected.get(item.color_id)
            if previous is None or area > previous[0]:
                selected[item.color_id] = (area, item)
        result.observations = [o for o in result.observations
                               if not o.color_id or selected[o.color_id][1] is o]
    if not preserve_status:
        flags = [o.valid for o in result.observations]
        result.status = ('NO_DETECTION' if not flags else 'OK' if all(flags) else
                         'PARTIAL' if any(flags) else 'NO_VALID_DETECTION')
        if any(w.startswith('HSV_UNCONFIGURED_COLORS:') for w in result.warnings):
            result.status = 'CONFIGURATION_INCOMPLETE'
    result.processing_ms = (time.perf_counter()-started)*1000
    return result


def _use_bbox(item, reason):
    item.valid, item.status = True, 'BBOX_ONLY'
    item.center_px, item.center_source = item.bbox_center_px, 'bbox'
    item.geometry_reason = reason


def _offset_box(box, x, y):
    return (box[0]+x,box[1]+y,box[2]+x,box[3]+y)


def _truncated(box, roi, tolerance):
    return (box[0] <= roi[0]+tolerance or box[1] <= roi[1]+tolerance
            or box[2] >= roi[2]-tolerance or box[3] >= roi[3]-tolerance)


def _validate_frame(frame, sequence, stamp):
    if (not isinstance(frame, np.ndarray) or frame.dtype != np.uint8 or frame.ndim != 3
            or frame.shape[2] != 3 or min(frame.shape[:2]) <= 0):
        raise ValueError('frame_bgr must be a nonempty uint8 HxWx3 BGR image')
    if type(sequence) is not int or sequence < 0:
        raise ValueError('frame_seq must be a nonnegative integer')
    if type(stamp) not in (int,float) or not math.isfinite(stamp):
        raise ValueError('stamp must be finite')


def _target(value):
    if type(value) is not int or not 0 <= value <= 6:
        raise ValueError('target_color_id must be 0～6')
    return value


def _search_roi(frame, roi):
    height,width = frame.shape[:2]
    if roi is None or isinstance(roi, (tuple,list)) and not roi:
        return (0,0,width,height)
    if (not isinstance(roi, (tuple,list)) or len(roi) != 4
            or any(type(v) is not int for v in roi)
            or not 0 <= roi[0] < roi[2] <= width or not 0 <= roi[1] < roi[3] <= height):
        raise ValueError('search_roi must be an integer ROI inside the image')
    return tuple(roi)


def _validate_detections(boxes):
    if not isinstance(boxes, (tuple,list)) or any(not isinstance(b, MaterialModelDetection) for b in boxes):
        raise ValueError('detections must be a list of MaterialModelDetection')
    return boxes
