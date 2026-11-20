"""Adapt the user's deployed X5 YOLOv8 model to original-image ring boxes."""

from dataclasses import dataclass
from pathlib import Path
import math
import sys


@dataclass(frozen=True)
class ModelDetection:
    """Postprocessed box in original-image pixels, with a semantic class label."""

    label: str
    confidence: float
    bbox_xyxy: tuple


class RingModelError(RuntimeError):
    def __init__(self, status, message):
        self.status = status
        super().__init__(f'{status}: {message}')


def load_ring_model(model_path='', model_format='', class_labels=('1', '2', '3'),
                    model_runtime_path='', confidence_threshold=0.75, nms_threshold=0.7):
    if not model_path.strip() or not model_format.strip():
        raise RingModelError('MODEL_NOT_CONFIGURED',
                             '请提供训练后的模型路径与格式，不使用示例权重。')
    model_path = Path(model_path).expanduser().resolve()
    if not model_path.is_file():
        raise RingModelError('MODEL_LOAD_FAILED', f'模型文件不存在：{model_path}')
    if model_format != 'rdk_yolov8_bin':
        raise RingModelError('MODEL_ADAPTER_NOT_IMPLEMENTED', f'不支持模型格式：{model_format}')
    if sorted(class_labels) != ['1', '2', '3']:
        raise ValueError('class_labels must map model indices to exactly 1, 2, 3')
    for name, value in (('confidence_threshold', confidence_threshold),
                        ('nms_threshold', nms_threshold)):
        if not math.isfinite(value) or not 0 <= value <= 1:
            raise ValueError(f'{name} must be finite and in [0, 1]')
    runtime = Path(model_runtime_path).expanduser().resolve()
    if not model_runtime_path or not (runtime / 'samples/vision/ultralytics_yolo/runtime/python/detect.py').is_file():
        raise RingModelError('MODEL_LOAD_FAILED', f'缺少已部署的 rdk_model_zoo：{runtime}')
    if str(runtime) not in sys.path:
        sys.path.insert(0, str(runtime))
    try:
        from samples.vision.ultralytics_yolo.runtime.python.detect import YoloDetect, YoloDetectConfig
        from samples.vision.ultralytics_yolo.runtime.python.cli import resolve_platform

        detector = YoloDetect(YoloDetectConfig(
            model_path=str(model_path), platform=resolve_platform('x5'), classes_num=3,
            resize_type=1, score_thres=confidence_threshold, nms_thres=nms_threshold,
        ))
    except Exception as error:
        raise RingModelError('MODEL_LOAD_FAILED', f'X5 YOLOv8 加载失败：{error}') from error
    return X5RingModel(detector, tuple(class_labels))


class X5RingModel:
    def __init__(self, detector, class_labels):
        self.detector = detector
        self.class_labels = class_labels

    def infer(self, frame_bgr):
        # The deployed runtime already performs letterbox inversion and classwise NMS.
        try:
            boxes, scores, class_ids = self.detector.predict(frame_bgr)
            return [ModelDetection(self.class_labels[int(class_id)], float(score),
                                   tuple(float(v) for v in box))
                    for box, score, class_id in zip(boxes, scores, class_ids)]
        except Exception as error:
            raise RingModelError('MODEL_INFERENCE_FAILED', str(error)) from error
