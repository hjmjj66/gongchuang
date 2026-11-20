"""Station identities and ROI geometry, reusable without a ROS node or camera."""

from dataclasses import asdict, dataclass, field
import math
import time

import numpy as np

from .ring_geometry import RingGeometry
from .ring_model import ModelDetection, RingModelError


@dataclass(frozen=True)
class DetectorConfig:
    confidence_threshold: float = 0.75
    roi_padding_ratio: float = 0.08
    minimum_roi_size_px: int = 20
    border_tolerance_px: float = 1.0
    allow_bbox_fallback: bool = True

    def __post_init__(self):
        if not math.isfinite(self.confidence_threshold) or not 0 <= self.confidence_threshold <= 1:
            raise ValueError('confidence_threshold must be in [0, 1]')
        for name in ('roi_padding_ratio', 'border_tolerance_px'):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0:
                raise ValueError(f'{name} must be nonnegative and finite')
        if type(self.minimum_roi_size_px) is not int or self.minimum_roi_size_px < 3:
            raise ValueError('minimum_roi_size_px must be an integer >= 3')
        if type(self.allow_bbox_fallback) is not bool:
            raise ValueError('allow_bbox_fallback must be boolean')


@dataclass
class RingObservation:
    ring_id: int | None
    identity_source: str
    valid: bool = False
    status: str = 'NOT_DETECTED'
    center_px: tuple | None = None
    center_source: str | None = None
    yolo_confidence: float | None = None
    bbox_xyxy: tuple | None = None
    bbox_center_px: tuple | None = None
    circle_center_px: tuple | None = None
    roi_xyxy: tuple | None = None
    radii_px: tuple = ()
    ellipses_px: tuple = ()
    circle_support: dict = field(default_factory=dict)
    geometry_reason: str | None = None


@dataclass
class RingFrameResult:
    frame_seq: int
    stamp: float
    image_size: tuple
    processing_ms: float
    status: str
    observations: tuple
    debug: dict = field(default_factory=dict, repr=False)
    stages_ms: dict = field(default_factory=dict)

    def to_dict(self):
        # Debug images remain local; they must not be copied into JSON output.
        return {
            'frame_seq': self.frame_seq, 'stamp': self.stamp,
            'timestamp_source': 'caller_supplied', 'image_size': self.image_size,
            'processing_ms': self.processing_ms, 'status': self.status,
            'stages_ms': self.stages_ms,
            'observations': [asdict(item) for item in self.observations],
        }


class RingDetector:
    def __init__(self, config=None, geometry_config=None, model=None, roi_executor=None):
        self.config = config or DetectorConfig()
        self.geometry = RingGeometry(geometry_config)
        self.model = model
        self.roi_executor = roi_executor

    def detect(self, frame, frame_seq=0, stamp=0.0, *, geometry_target=0):
        """Model adapter implements infer(BGR) -> iterable[ModelDetection]."""
        _validate_frame(frame, frame_seq, stamp)
        _validate_geometry_target(geometry_target)
        if self.model is None:
            raise RingModelError('MODEL_NOT_CONFIGURED', '没有接入模型；请使用显式手动 ROI 调试。')
        started = time.perf_counter()
        boxes = self.model.infer(frame)
        model_ms = (time.perf_counter() - started) * 1000
        result = self._process_boxes(frame, boxes, frame_seq, stamp, started, geometry_target)
        result.stages_ms = {'yolo': model_ms, 'geometry': result.processing_ms - model_ms}
        return result

    def detect_boxes(self, frame, detections, frame_seq=0, stamp=0.0, *, geometry_target=0):
        """Use actual postprocessed model boxes; does not create synthetic identities."""
        _validate_frame(frame, frame_seq, stamp)
        _validate_geometry_target(geometry_target)
        return self._process_boxes(frame, detections, frame_seq, stamp, time.perf_counter(), geometry_target)

    def _process_boxes(self, frame, detections, frame_seq, stamp, started, geometry_target=0):
        selected = select_best_boxes(detections, self.config.confidence_threshold)
        pending = {}
        if self.roi_executor is not None:
            pending = {ring_id: self.roi_executor.submit(
                self._locate, frame, box.bbox_xyxy, ring_id, 'yolo', float(box.confidence),
                not geometry_target or ring_id == geometry_target)
                for ring_id, box in selected.items()}
        observations, debug = [], {}
        for ring_id in (1, 2, 3):
            box = selected.get(ring_id)
            if box is None:
                observations.append(RingObservation(ring_id, 'yolo'))
                continue
            if self.roi_executor is None:
                observation, geometry = self._locate(
                    frame, box.bbox_xyxy, ring_id, 'yolo', float(box.confidence),
                    not geometry_target or ring_id == geometry_target)
            else:
                observation, geometry = pending[ring_id].result()
            observations.append(observation)
            if geometry is not None:
                debug[ring_id] = geometry
        return self._frame_result(frame, frame_seq, stamp, started, observations, debug)

    def detect_roi(self, frame, bbox_xyxy, ring_id=None, frame_seq=0, stamp=0.0):
        """Explicit manual identity for image-only geometry debugging."""
        _validate_frame(frame, frame_seq, stamp)
        _validate_box(bbox_xyxy)
        if ring_id not in (None, 1, 2, 3):
            raise ValueError('manual ring_id must be None, 1, 2 or 3')
        started = time.perf_counter()
        observation, geometry = self._locate(
            frame, tuple(float(v) for v in bbox_xyxy), ring_id, 'manual', None,
        )
        debug = {ring_id: geometry} if geometry is not None else {}
        return self._frame_result(frame, frame_seq, stamp, started, [observation], debug)

    def _locate(self, frame, box, ring_id, identity_source, confidence, refine=True):
        height, width = frame.shape[:2]
        config = self.config
        x1, y1, x2, y2 = box
        pad_x = (x2 - x1) * config.roi_padding_ratio
        pad_y = (y2 - y1) * config.roi_padding_ratio
        rx1, ry1 = max(0, math.floor(x1 - pad_x)), max(0, math.floor(y1 - pad_y))
        rx2, ry2 = min(width, math.ceil(x2 + pad_x)), min(height, math.ceil(y2 + pad_y))
        observation = RingObservation(
            ring_id, identity_source, yolo_confidence=confidence, bbox_xyxy=box,
            bbox_center_px=((x1 + x2) / 2, (y1 + y2) / 2),
        )
        if rx2 <= rx1 or ry2 <= ry1:
            observation.status = 'BOX_OUTSIDE_IMAGE'
            return observation, None
        observation.roi_xyxy = (rx1, ry1, rx2, ry2)
        truncated = (x1 <= config.border_tolerance_px or y1 <= config.border_tolerance_px
                     or x2 >= width - config.border_tolerance_px
                     or y2 >= height - config.border_tolerance_px)
        if not refine:
            observation.geometry_reason = 'NOT_TARGET'
            if truncated:
                observation.status = 'TRUNCATED'
            else:
                observation.valid, observation.status = True, 'BBOX_ONLY'
                observation.center_px, observation.center_source = observation.bbox_center_px, 'bbox'
            return observation, None
        geometry = None
        if min(rx2 - rx1, ry2 - ry1) < config.minimum_roi_size_px:
            observation.geometry_reason = 'ROI_TOO_SMALL'
        else:
            geometry = self.geometry.detect(
                frame[ry1:ry2, rx1:rx2], (x1 - rx1, y1 - ry1, x2 - rx1, y2 - ry1),
            )
            observation.geometry_reason = geometry.reason
        if geometry is not None and geometry.center_px is not None:
            u, v = geometry.center_px
            observation.valid = True
            observation.status = 'OK'
            observation.center_px = (rx1 + u, ry1 + v)
            observation.circle_center_px = observation.center_px
            observation.center_source = 'circle'
            observation.radii_px = geometry.radii_px
            observation.ellipses_px = tuple(
                ((rx1 + c.x, ry1 + c.y), (2 * c.ellipse_axes[0], 2 * c.ellipse_axes[1]),
                 c.ellipse_angle)
                for c in geometry.selected if c.ellipse_axes is not None)
            observation.circle_support = geometry.support
        elif truncated:
            observation.status = 'TRUNCATED'
        elif config.allow_bbox_fallback:
            observation.valid = True
            observation.status = 'BBOX_ONLY'
            observation.center_px = observation.bbox_center_px
            observation.center_source = 'bbox'
        else:
            observation.status = 'GEOMETRY_FAILED'
        return observation, geometry

    @staticmethod
    def _frame_result(frame, frame_seq, stamp, started, observations, debug):
        valid = [item for item in observations if item.valid]
        status = 'NO_DETECTION'
        if valid:
            status = 'OK' if len(valid) == len(observations) else 'PARTIAL'
        return RingFrameResult(
            frame_seq, float(stamp), (frame.shape[1], frame.shape[0]),
            (time.perf_counter() - started) * 1000, status, tuple(observations), debug,
        )


def _validate_geometry_target(value):
    if type(value) is not int or not 0 <= value <= 3:
        raise ValueError('geometry_target must be 0～3 (0 refines all rings)')


def select_best_boxes(detections, confidence_threshold):
    selected = {}
    for detection in detections:
        if not isinstance(detection, ModelDetection):
            raise ValueError('model output must contain ModelDetection objects')
        if detection.label not in ('1', '2', '3'):
            continue
        if not math.isfinite(detection.confidence) or not 0 <= detection.confidence <= 1:
            raise ValueError('model confidence must be finite and in [0, 1]')
        _validate_box(detection.bbox_xyxy)
        if detection.confidence <= confidence_threshold:
            continue
        ring_id = int(detection.label)
        if ring_id not in selected or detection.confidence > selected[ring_id].confidence:
            selected[ring_id] = ModelDetection(
                detection.label, float(detection.confidence),
                tuple(float(v) for v in detection.bbox_xyxy),
            )
    return selected


def _validate_box(box):
    if len(box) != 4 or not all(math.isfinite(v) for v in box):
        raise ValueError('bbox_xyxy must contain four finite coordinates')
    if box[2] <= box[0] or box[3] <= box[1]:
        raise ValueError('bbox_xyxy must have positive width and height')


def _validate_frame(frame, frame_seq, stamp):
    if not isinstance(frame, np.ndarray) or frame.dtype != np.uint8 or (
            frame.ndim != 3 or frame.shape[2] != 3 or frame.size == 0):
        raise ValueError('frame must be a nonempty uint8 BGR image')
    if type(frame_seq) is not int or frame_seq < 0 or not math.isfinite(stamp):
        raise ValueError('frame_seq must be nonnegative integer and stamp must be finite')
