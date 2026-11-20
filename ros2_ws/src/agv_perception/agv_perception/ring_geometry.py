"""Circle geometry inside a single station ROI, independent of ROS and YOLO."""

from dataclasses import dataclass, field
import math

import cv2
import numpy as np


@dataclass(frozen=True)
class GeometryConfig:
    method: str = 'contour'
    min_ellipse_axis_ratio: float = 0.65
    max_contour_fit_error_px: float = 2.0
    min_contour_area_ratio: float = 0.6
    contour_adaptive_block_size: int = 31
    contour_adaptive_c: float = 5.0
    contour_max_side_px: int = 192
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
        if self.method not in ('contour', 'hough'):
            raise ValueError('geometry.method must be contour or hough')
        if self.preprocess not in ('gray', 'binary_morphology'):
            raise ValueError('geometry.preprocess must be gray or binary_morphology')
        integer_fields = (
            'blur_kernel', 'binary_threshold', 'morphology_kernel',
            'open_iterations', 'close_iterations', 'circumference_samples',
            'radial_tolerance_px', 'angle_sectors', 'min_supported_sectors',
            'contour_adaptive_block_size',
            'contour_max_side_px',
        )
        for name in integer_fields:
            if type(getattr(self, name)) is not int:
                raise ValueError(f'geometry.{name} must be an integer')
        for name in ('blur_kernel', 'morphology_kernel'):
            value = getattr(self, name)
            if value < 1 or value % 2 != 1:
                raise ValueError(f'geometry.{name} must be positive and odd')
        if self.contour_adaptive_block_size < 3 or self.contour_adaptive_block_size % 2 != 1:
            raise ValueError('geometry.contour_adaptive_block_size must be odd and >= 3')
        if self.contour_max_side_px < 0:
            raise ValueError('geometry.contour_max_side_px must be nonnegative')
        if not 0 <= self.binary_threshold <= 255:
            raise ValueError('geometry.binary_threshold must be between 0 and 255')
        for name in ('open_iterations', 'close_iterations', 'radial_tolerance_px'):
            if getattr(self, name) < 0:
                raise ValueError(f'geometry.{name} must be nonnegative')
        positive_fields = (
            'hough_dp', 'hough_min_distance_px', 'hough_param1', 'hough_param2',
            'min_radius_ratio', 'max_radius_ratio', 'max_center_offset_ratio',
            'edge_low_threshold', 'edge_high_threshold', 'merge_distance_ratio',
            'max_contour_fit_error_px',
        )
        for name in positive_fields:
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'geometry.{name} must be positive and finite')
        for name in ('blur_sigma', 'duplicate_radius_tolerance_px', 'group_ambiguity_margin',
                     'contour_adaptive_c'):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0:
                raise ValueError(f'geometry.{name} must be nonnegative and finite')
        if self.min_radius_ratio >= self.max_radius_ratio:
            raise ValueError('geometry radius ratios must be increasing')
        if self.edge_low_threshold >= self.edge_high_threshold:
            raise ValueError('geometry edge thresholds must be increasing')
        for name in ('min_edge_support', 'sector_support_threshold',
                     'min_ellipse_axis_ratio', 'min_contour_area_ratio'):
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
    ellipse_axes: tuple | None = None
    ellipse_angle: float = 0.0


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
    def __init__(self, config=None, *, contour_boundary='hole'):
        self.config = config or GeometryConfig()
        self.contour_boundary = contour_boundary
        angles = np.arange(self.config.circumference_samples) * (
            2 * np.pi / self.config.circumference_samples)
        self._cos = np.cos(angles)
        self._sin = np.sin(angles)
        self._radial_offsets = np.arange(-self.config.radial_tolerance_px,
                                         self.config.radial_tolerance_px + 1)
        sectors = np.array_split(np.arange(self.config.circumference_samples),
                                 self.config.angle_sectors)
        self._sector_starts = np.array([part[0] for part in sectors])
        self._sector_sizes = np.array([len(part) for part in sectors])

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
        if config.method == 'contour':
            # Rings use enclosed holes; solid material caps use the outer boundary.
            contours_image = processed
            image_scale = (1.0, 1.0)
            if config.preprocess == 'gray':
                height, width = blurred.shape
                threshold_input = blurred
                if config.contour_max_side_px and max(width, height) > config.contour_max_side_px:
                    ratio = config.contour_max_side_px / max(width, height)
                    size = (max(1, round(width * ratio)), max(1, round(height * ratio)))
                    threshold_input = cv2.resize(blurred, size, interpolation=cv2.INTER_LINEAR)
                    image_scale = (size[0] / width, size[1] / height)
                block = max(3, round(config.contour_adaptive_block_size * min(image_scale)) | 1)
                contours_image = cv2.adaptiveThreshold(
                    threshold_input, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV,
                    block, config.contour_adaptive_c)
            candidates = self._contour_candidates(
                contours_image, edges, reference, scale, min_radius, max_radius, image_scale)
        else:
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

    def _contour_candidates(self, image, edges, reference, scale, min_radius, max_radius,
                            image_scale):
        config = self.config
        sx, sy = image_scale
        contours, hierarchy = cv2.findContours(image, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
        ellipses = []
        for index, contour in enumerate(contours):
            is_hole = hierarchy[0, index, 3] >= 0
            if is_hole != (self.contour_boundary == 'hole'):
                continue
            if len(contour) < max(20, min_radius * 3 * min(sx, sy)):
                continue
            _, _, width, height = cv2.boundingRect(contour)
            if min(width / sx, height / sy) < 2 * min_radius:
                continue
            area = abs(cv2.contourArea(contour)) / (sx * sy)
            if area < math.pi * min_radius ** 2 * config.min_contour_area_ratio:
                continue
            # Long contours need no more than 64 evenly spaced points for fitting.
            # Copy because OpenCV requires a contiguous point array after striding.
            points = contour[::max(1, math.ceil(len(contour) / 64))].copy()
            if image_scale != (1.0, 1.0):
                # Invert resize's pixel-center mapping before fitting, including
                # the independently rounded width/height. Validation stays full size.
                points = points.astype(np.float32)
                points[:, 0, 0] = (points[:, 0, 0] + 0.5) / sx - 0.5
                points[:, 0, 1] = (points[:, 0, 1] + 0.5) / sy - 0.5
            (x, y), (width, height), angle = cv2.fitEllipseDirect(points)
            a, b = width / 2, height / 2
            radius = (a + b) / 2
            if not (min_radius <= radius <= max_radius and
                    min(a, b) >= max(a, b) * config.min_ellipse_axis_ratio):
                continue
            if area < math.pi * a * b * config.min_contour_area_ratio:
                continue
            radians = math.radians(angle)
            cosine, sine = math.cos(radians), math.sin(radians)
            offsets = points[:, 0, :] - (x, y)
            u = (offsets[:, 0] * cosine + offsets[:, 1] * sine) / a
            v = (-offsets[:, 0] * sine + offsets[:, 1] * cosine) / b
            residual = np.abs(np.hypot(u, v) - 1) * min(a, b)
            if residual.mean() > config.max_contour_fit_error_px:
                continue
            ellipses.append((x, y, a, b, angle))
        if not ellipses:
            return ()
        values = np.asarray(ellipses, dtype=np.float64)
        radians = np.deg2rad(values[:, 4, None, None])
        cosine, sine = np.cos(radians), np.sin(radians)
        a = values[:, 2, None, None] + self._radial_offsets[None, :, None]
        b = values[:, 3, None, None] + self._radial_offsets[None, :, None]
        xs = np.rint(values[:, 0, None, None] + a * self._cos * cosine
                     - b * self._sin * sine).astype(int)
        ys = np.rint(values[:, 1, None, None] + a * self._cos * sine
                     + b * self._sin * cosine).astype(int)
        supports, sectors, alignments = self._sample_support(edges, xs, ys)
        candidates = []
        for (x, y, a, b, angle), support, sector_count, alignment in zip(
                ellipses, supports, sectors, alignments):
            reason = 'OK'
            if math.dist((x, y), reference) > scale * config.max_center_offset_ratio:
                reason = 'CENTER_OUTSIDE_RANGE'
            elif support < config.min_edge_support:
                reason = 'WEAK_EDGE_SUPPORT'
            elif sector_count < config.min_supported_sectors:
                reason = 'INSUFFICIENT_ANGLE_COVERAGE'
            candidates.append(CircleCandidate(
                x, y, (a + b) / 2, float(support), int(sector_count),
                reason == 'OK', reason, float(alignment), (a, b), angle))
        return tuple(candidates)

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
        # Dense dashed rings can produce thousands of candidates. Batch sampling
        # avoids a Python/NumPy reduction per circle and bounds temporary arrays.
        for start in range(0, len(circles[0]), 128):
            batch = np.asarray(circles[0][start:start + 128], dtype=np.float64)
            supports, sector_counts, alignments = self._edge_support_batch(edges, batch)
            for (x, y, radius), support, sectors, alignment in zip(
                    batch, supports, sector_counts, alignments):
                x, y, radius = float(x), float(y), float(radius)
                support, sectors, alignment = float(support), int(sectors), float(alignment)
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
        supports, sectors, alignments = self._edge_support_batch(
            edges, np.array([[x, y, radius]], dtype=np.float64),
        )
        return float(supports[0]), int(sectors[0]), float(alignments[0])

    def _edge_support_batch(self, edges, circles):
        config = self.config
        radial = circles[:, 2, None] + self._radial_offsets
        xs = np.rint(circles[:, 0, None, None]
                     + radial[:, :, None] * self._cos).astype(int)
        ys = np.rint(circles[:, 1, None, None]
                     + radial[:, :, None] * self._sin).astype(int)
        return self._sample_support(edges, xs, ys)

    def _sample_support(self, edges, xs, ys):
        config = self.config
        height, width = edges.shape
        inside = (xs >= 0) & (xs < width) & (ys >= 0) & (ys < height)
        hits = np.zeros(xs.shape, dtype=bool)
        hits[inside] = edges[ys[inside], xs[inside]] > 0
        # Out-of-frame arc samples count as missing support, not as invisible successes.
        supported = hits.any(axis=1)
        sector_ratios = np.add.reduceat(supported, self._sector_starts, axis=1) / self._sector_sizes
        sectors = (sector_ratios >= config.sector_support_threshold).sum(axis=1)
        nearest = np.where(hits, np.abs(self._radial_offsets)[None, :, None],
                           config.radial_tolerance_px + 1).min(axis=1)
        alignment = (1 - nearest / (config.radial_tolerance_px + 1)).mean(axis=1)
        return supported.mean(axis=1), sectors, alignment

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
            weight_sum = sum(c.support for c in group)
            center = (sum(c.x * c.support for c in group) / weight_sum,
                      sum(c.y * c.support for c in group) / weight_sum)
            support = sum(c.support * c.support for c in group) / weight_sum
            quality = sum(_circle_quality(c) * c.support for c in group) / weight_sum
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
             'edge_alignment': sum((c.edge_alignment if c.edge_alignment is not None else c.support)
                                   * c.support for c in group) / sum(c.support for c in group),
             'min_supported_sectors': min(c.supported_sectors for c in group)},
            'OK', tuple(candidates), tuple(group),
        )


def _circle_quality(candidate):
    """Prefer edges on the circle over edges only found near the tolerance boundary."""
    alignment = candidate.edge_alignment
    return candidate.support if alignment is None else (candidate.support + alignment) / 2
