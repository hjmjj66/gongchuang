"""Six-class material adapter for the locally deployed X5 YOLO runtime."""

from dataclasses import dataclass
import importlib
import math
from pathlib import Path
import sys

from .material_config import TOP_LABELS, WHOLE_LABELS


class MaterialModelError(RuntimeError):
    def __init__(self, status, message):
        self.status = status
        super().__init__(f'{status}: {message}')


@dataclass(frozen=True)
class MaterialModelDetection:
    color_id: int
    raw_label: str
    confidence: float
    bbox_xyxy: tuple

    def __post_init__(self):
        if type(self.color_id) is not int or not 1 <= self.color_id <= 6:
            raise ValueError('color_id must be an integer 1～6')
        if not isinstance(self.raw_label, str):
            raise ValueError('raw_label must be a string')
        if (type(self.confidence) not in (int, float) or not math.isfinite(self.confidence)
                or not 0 <= self.confidence <= 1):
            raise ValueError('confidence must be finite and in [0,1]')
        box = self.bbox_xyxy
        if (not isinstance(box, (tuple, list)) or len(box) != 4
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in box)
                or box[2] <= box[0] or box[3] <= box[1]):
            raise ValueError('bbox_xyxy must be finite with positive width and height')


def load_material_model(config):
    if config['debug_mode'] == 'geometry_only':
        return None
    if config['test_combine']:
        from .material_combine import CombinedMaterialModel
        try:
            return CombinedMaterialModel(config)
        except Exception as error:
            raise MaterialModelError('MODEL_LOAD_FAILED', str(error)) from error
    if config['use_jhb_model']:
        from .material_jhb import JhbMaterialModel
        try:
            return JhbMaterialModel(config)
        except Exception as error:
            raise MaterialModelError('MODEL_LOAD_FAILED', str(error)) from error
    if config['material_backend'] != 2:
        return None
    branch = 'top' if config['material_model'] == 1 else 'whole'
    model_path = config[branch + '.model_path']
    if not model_path.strip():
        raise MaterialModelError('MODEL_NOT_CONFIGURED', f'{branch}.model_path is empty')
    model_path = Path(model_path).expanduser().resolve()
    runtime = Path(config['model_runtime_path']).expanduser().resolve()
    labels = tuple(config[branch + '.class_labels'])
    try:
        if not model_path.is_file():
            raise ValueError(f'模型文件不存在：{model_path}')
        actual = Path(config[branch + '.classes_path']).expanduser().read_text().splitlines()
        if tuple(actual) != labels:
            raise ValueError(f'类别文件与 {branch}.class_labels 顺序不一致：{actual}')
        entry = runtime / 'samples/vision/ultralytics_yolo/runtime/python/detect.py'
        if not entry.is_file():
            raise ValueError(f'缺少部署运行库：{entry}')
        # Same-named modules from another deployment must not silently select a different runtime.
        for name in ('samples', 'utils'):
            module = sys.modules.get(name)
            if module is not None:
                locations = list(getattr(module, '__path__', []))
                if getattr(module, '__file__', None):
                    locations.append(module.__file__)
                if any(not Path(location).resolve().is_relative_to(runtime) for location in locations):
                    raise ValueError(f'{name} 已从其他运行库导入；请用独立进程或统一运行库')
        if str(runtime) not in sys.path:
            sys.path.insert(0, str(runtime))
        detect = importlib.import_module('samples.vision.ultralytics_yolo.runtime.python.detect')
        cli = importlib.import_module('samples.vision.ultralytics_yolo.runtime.python.cli')
        detector = detect.YoloDetect(detect.YoloDetectConfig(
            model_path=str(model_path), platform=cli.resolve_platform('x5'), classes_num=6,
            resize_type=1, score_thres=config['confidence_threshold'],
            nms_thres=config['nms_threshold']))
    except Exception as error:
        raise MaterialModelError('MODEL_LOAD_FAILED', str(error)) from error
    return X5MaterialModel(detector, labels)


class X5MaterialModel:
    def __init__(self, detector, class_labels):
        self.detector = detector
        self.class_labels = tuple(class_labels)
        vocabulary = TOP_LABELS if self.class_labels == TOP_LABELS else WHOLE_LABELS
        self.color_ids = {label: i + 1 for i, label in enumerate(vocabulary)}

    def infer(self, image_bgr):
        try:
            # predict already applies classwise NMS and reverses the letterbox transform.
            boxes, scores, class_ids = self.detector.predict(image_bgr)
            if not len(boxes) == len(scores) == len(class_ids):
                raise ValueError('runtime returned arrays of different lengths')
            result = []
            for box, score, index in zip(boxes, scores, class_ids):
                if not math.isfinite(float(index)) or int(index) != index or not 0 <= int(index) < 6:
                    raise ValueError(f'invalid runtime class index: {index}')
                label = self.class_labels[int(index)]
                result.append(MaterialModelDetection(
                    self.color_ids[label], label, float(score), tuple(float(v) for v in box)))
            return result
        except Exception as error:
            raise MaterialModelError('MODEL_INFERENCE_FAILED', str(error)) from error
