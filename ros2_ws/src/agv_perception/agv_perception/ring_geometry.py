"""Circle geometry inside a single station ROI, independent of ROS and YOLO."""

from dataclasses import dataclass, field
import math

import cv2
import numpy as np


@dataclass(frozen=True)
class GeometryConfig:
    preprocess: str = 'gray'
    blur_kernel: int = 5
    blur_sigma: float = 1.2
    binary_threshold: int = 100
    morphology_kernel: int = 3
    open_iterations: int = 0
    close_iterations: int = 1
    hough_dp: float = 1.0
    hough_min_distance_px: float = 4.0
    hough_param1: float = 100.0
    hough_param2: float = 25.0
    min_radius_ratio: float = 0.12
    max_radius_ratio: float = 0.55
    max_center_offset_ratio: float = 0.25
    edge_low_threshold: float = 50.0
    edge_high_threshold: float = 100.0
    circumference_samples: int = 180
    radial_tolerance_px: int = 3
    min_edge_support: float = 0.55
    angle_sectors: int = 12
    sector_support_threshold: float = 0.4
    min_supported_sectors: int = 8
    merge_distance_ratio: float = 0.04
    duplicate_radius_tolerance_px: float = 2.0
    group_ambiguity_margin: float = 0.04

    def __post_init__(self):
        if self.preprocess not in ('gray', 'binary_morphology'):
            raise ValueError('geometry.preprocess must be gray or binary_morphology')
        integer_fields = (
            'blur_kernel', 'binary_threshold', 'morphology_kernel',
            'open_iterations', 'close_iterations', 'circumference_samples',
            'radial_tolerance_px', 'angle_sectors', 'min_supported_sectors',
        )
        for name in integer_fields:
            if type(getattr(self, name)) is not int:
                raise ValueError(f'geometry.{name} must be an integer')
        for name in ('blur_kernel', 'morphology_kernel'):
            value = getattr(self, name)
            if value < 1 or value % 2 != 1:
                raise ValueError(f'geometry.{name} must be positive and odd')
        if not 0 <= self.binary_threshold <= 255:
            raise ValueError('geometry.binary_threshold must be between 0 and 255')
        for name in ('open_iterations', 'close_iterations', 'radial_tolerance_px'):
            if getattr(self, name) < 0:
                raise ValueError(f'geometry.{name} must be nonnegative')
        positive_fields = (
            'hough_dp', 'hough_min_distance_px', 'hough_param1', 'hough_param2',
            'min_radius_ratio', 'max_radius_ratio', 'max_center_offset_ratio',
            'edge_low_threshold', 'edge_high_threshold', 'merge_distance_ratio',
        )
        for name in positive_fields:
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'geometry.{name} must be positive and finite')
        for name in ('blur_sigma', 'duplicate_radius_tolerance_px', 'group_ambiguity_margin'):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0:
                raise ValueError(f'geometry.{name} must be nonnegative and finite')
        if self.min_radius_ratio >= self.max_radius_ratio:
            raise ValueError('geometry radius ratios must be increasing')
        if self.edge_low_threshold >= self.edge_high_threshold:
            raise ValueError('geometry edge thresholds must be increasing')
        for name in ('min_edge_support', 'sector_support_threshold'):
            value = getattr(self, name)
            if not math.isfinite(value) or not 0 < value <= 1:
                raise ValueError(f'geometry.{name} must be in (0, 1]')
        if not 1 <= self.min_supported_sectors <= self.angle_sectors:
            raise ValueError('geometry supported sector count is out of range')
        if self.circumference_samples < self.angle_sectors:
            raise ValueError('geometry needs at least one sample per angle sector')


@dataclass(frozen=True)
class CircleCandidate:
    x: float
    y: float
    radius: float
    support: float
    supported_sectors: int
    accepted: bool = True
    reason: str = 'OK'
    edge_alignment: float | None = None


@dataclass
class GeometryResult:
    center_px: tuple | None
    radii_px: tuple = ()
    support: dict = field(default_factory=dict)
    reason: str = 'NO_CIRCLE'
    candidates: tuple = ()
    selected: tuple = ()
    processed: np.ndarray | None = field(default=None, repr=False)
    edges: np.ndarray | None = field(default=None, repr=False)


class RingGeometry:
    def __init__(self, config=None):
        self.config = config or GeometryConfig()

    def detect(self, roi, local_box):
        """Process a validated BGR crop and its original box in crop coordinates."""
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        config = self.config
        blurred = cv2.GaussianBlur(
            gray, (config.blur_kernel, config.blur_kernel), config.blur_sigma,
        )
        processed = self._preprocess(blurred)
        edges = cv2.Canny(blurred, config.edge_low_threshold, config.edge_high_threshold)
        x1, y1, x2, y2 = local_box
        scale = min(x2 - x1, y2 - y1)
        reference = ((x1 + x2) / 2, (y1 + y2) / 2)
        min_radius = max(1, math.ceil(scale * config.min_radius_ratio))
        max_radius = math.floor(scale * config.max_radius_ratio)
        if max_radius <= min_radius:
            return GeometryResult(None, reason='RADIUS_RANGE_TOO_SMALL',
                                  processed=processed, edges=edges)
        circles = cv2.HoughCircles(
            processed, cv2.HOUGH_GRADIENT, config.hough_dp,
            config.hough_min_distance_px, param1=config.hough_param1,
            param2=config.hough_param2, minRadius=min_radius, maxRadius=max_radius,
        )
        candidates = self._evaluate(circles, edges, reference, scale)
        result = self.fuse_candidates(candidates, reference, scale)
        result.processed = processed
        result.edges = edges
        return result

    def _preprocess(self, gray):
        config = self.config
        if config.preprocess == 'gray':
            return gray
        _, mask = cv2.threshold(gray, config.binary_threshold, 255, cv2.THRESH_BINARY_INV)
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (config.morphology_kernel, config.morphology_kernel),
        )
        if config.open_iterations:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel,
                                    iterations=config.open_iterations)
        if config.close_iterations:
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel,
                                    iterations=config.close_iterations)
        return mask

    def _evaluate(self, circles, edges, reference, scale):
        if circles is None:
            return ()
        config = self.config
        candidates = []
        for x, y, radius in circles[0]:
            x, y, radius = float(x), float(y), float(radius)
            support, sectors, alignment = self._edge_support(edges, x, y, radius)
            reason = 'OK'
            if math.hypot(x - reference[0], y - reference[1]) > (
                    scale * config.max_center_offset_ratio):
                reason = 'CENTER_OUTSIDE_RANGE'
            elif support < config.min_edge_support:
                reason = 'WEAK_EDGE_SUPPORT'
            elif sectors < config.min_supported_sectors:
                reason = 'INSUFFICIENT_ANGLE_COVERAGE'
            candidates.append(CircleCandidate(
                x, y, radius, support, sectors, reason == 'OK', reason, alignment,
            ))
        return tuple(candidates)

    def _edge_support(self, edges, x, y, radius):
        config = self.config
        angles = np.arange(config.circumference_samples) * (
            2 * np.pi / config.circumference_samples)
        radial = radius + np.arange(-config.radial_tolerance_px,
                                    config.radial_tolerance_px + 1)
        xs = np.rint(x + radial[:, None] * np.cos(angles)).astype(int)
        ys = np.rint(y + radial[:, None] * np.sin(angles)).astype(int)
        height, width = edges.shape
        inside = (xs >= 0) & (xs < width) & (ys >= 0) & (ys < height)
        hits = np.zeros(xs.shape, dtype=bool)
        hits[inside] = edges[ys[inside], xs[inside]] > 0
        # Out-of-frame arc samples count as missing support, not as invisible successes.
        supported = hits.any(axis=0)
        sectors = sum(float(part.mean()) >= config.sector_support_threshold
                      for part in np.array_split(supported, config.angle_sectors))
        offsets = np.abs(np.arange(-config.radial_tolerance_px,
                                   config.radial_tolerance_px + 1))[:, None]
        nearest = np.where(hits, offsets, config.radial_tolerance_px + 1).min(axis=0)
        alignment = float((1 - nearest / (config.radial_tolerance_px + 1)).mean())
        return float(supported.mean()), sectors, alignment

    def fuse_candidates(self, candidates, reference, scale):
        """Group evaluated candidates without allowing chains of distant centers."""
        config = self.config
        distance_limit = scale * config.merge_distance_ratio
        groups = []
        for candidate in sorted((c for c in candidates if c.accepted),
                                key=_circle_quality, reverse=True):
            for group in groups:
                if all(math.hypot(candidate.x - other.x, candidate.y - other.y)
                       <= distance_limit for other in group):
                    if not any(abs(candidate.radius - other.radius)
                               <= config.duplicate_radius_tolerance_px for other in group):
                        group.append(candidate)
                    break
            else:
                groups.append([candidate])
        if not groups:
            reason = 'NO_CIRCLE' if not candidates else 'NO_TRUSTED_CIRCLE'
            return GeometryResult(None, reason=reason, candidates=tuple(candidates))
        ranked = []
        for group in groups:
            weights = np.array([c.support for c in group])
            center = tuple(float(v) for v in np.average(
                [(c.x, c.y) for c in group], axis=0, weights=weights,
            ))
            support = float(np.average(weights, weights=weights))
            quality = float(np.average([_circle_quality(c) for c in group], weights=weights))
            distance = math.hypot(center[0] - reference[0], center[1] - reference[1]) / scale
            score = quality - 0.3 * distance + 0.01 * min(len(group) - 1, 2)
            ranked.append((score, center, support, group))
        ranked.sort(key=lambda item: item[0], reverse=True)
        if len(ranked) > 1 and ranked[0][0] - ranked[1][0] <= config.group_ambiguity_margin:
            return GeometryResult(None, reason='AMBIGUOUS_CIRCLE_GROUPS',
                                  candidates=tuple(candidates))
        _, center, support, group = ranked[0]
        return GeometryResult(
            center, tuple(c.radius for c in group),
            {'edge_ratio': support, 'circle_count': len(group),
             'edge_alignment': float(np.average(
                 [c.edge_alignment if c.edge_alignment is not None else c.support for c in group],
                 weights=[c.support for c in group],
             )),
             'min_supported_sectors': min(c.supported_sectors for c in group)},
            'OK', tuple(candidates), tuple(group),
        )


def _circle_quality(candidate):
    """Prefer edges on the circle over edges only found near the tolerance boundary."""
    alignment = candidate.edge_alignment
    return candidate.support if alignment is None else (candidate.support + alignment) / 2
