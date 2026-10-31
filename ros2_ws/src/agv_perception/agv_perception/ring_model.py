"""Boundary for a future trained model; no inference runtime or weights bundled."""

from dataclasses import dataclass
from pathlib import Path


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


def load_ring_model(model_path='', model_format='', class_labels=('1', '2', '3')):
    """Leave deployment selection until the user's trained model is available."""
    if not model_path.strip() or not model_format.strip():
        raise RingModelError('MODEL_NOT_CONFIGURED',
                             '请提供训练后的模型路径与格式；当前均留空，不使用示例权重。')
    if not Path(model_path).is_file():
        raise RingModelError('MODEL_LOAD_FAILED', f'模型文件不存在：{model_path}')
    raise RingModelError(
        'MODEL_ADAPTER_NOT_IMPLEMENTED',
        f'尚未接入 {model_format} 推理运行时；收到真实模型后实现 infer(frame_bgr)，'
        f'将类别 {list(class_labels)} 的框还原为原图坐标。',
    )
