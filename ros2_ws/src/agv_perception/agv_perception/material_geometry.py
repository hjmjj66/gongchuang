"""Local Hough-circle localization for solid material top surfaces."""

from dataclasses import dataclass, field
import math

import cv2
import numpy as np


@dataclass
class TopCircleResult:
    reason: str
    center: tuple = None
    radius: float = None
    support: float = None
    debug: dict = field(default_factory=dict)


class TopCircleLocator:
    def __init__(self, config):
        self.p = {k.removeprefix('top_circle.'): v for k, v in config.items()
                  if k.startswith('top_circle.')}

    def detect(self, roi_bgr, local_box):
        p = self.p
        gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (p['blur_kernel'], p['blur_kernel']), p['blur_sigma'])
        edges = cv2.Canny(gray, p['hough_param1'] / 2, p['hough_param1'])
        x1, y1, x2, y2 = local_box
        side = min(x2-x1, y2-y1)
        min_r = max(1, math.ceil(side * p['min_radius_ratio']))
        max_r = math.floor(side * p['max_radius_ratio'])
        debug = {'gray': gray, 'edges': edges, 'candidates': []}
        if max_r < min_r:
            return TopCircleResult('RADIUS_RANGE_EMPTY', debug=debug)
        circles = cv2.HoughCircles(
            gray, cv2.HOUGH_GRADIENT, dp=p['hough_dp'],
            minDist=p['hough_min_distance_px'], param1=p['hough_param1'],
            param2=p['hough_param2'], minRadius=min_r, maxRadius=max_r)
        if circles is None:
            return TopCircleResult('NO_CIRCLE', debug=debug)
        height, width = gray.shape
        box_center = ((x1+x2)/2, (y1+y2)/2)
        angles = np.arange(72) * (2 * np.pi / 72)
        offsets = np.arange(-p['edge_radial_tolerance_px'], p['edge_radial_tolerance_px'] + 1)
        accepted = []
        for x, y, r in circles[0].astype(float):
            distance = math.hypot(x-box_center[0], y-box_center[1])
            reason = None
            if distance > side * p['max_center_offset_ratio']:
                reason = 'CENTER_OFFSET'
            elif x-r < 0 or y-r < 0 or x+r > width-1 or y+r > height-1:
                reason = 'CIRCLE_CLIPPED'
            xs = np.rint(x + np.cos(angles)[:, None] * (r + offsets)).astype(int)
            ys = np.rint(y + np.sin(angles)[:, None] * (r + offsets)).astype(int)
            inside = (xs >= 0) & (ys >= 0) & (xs < width) & (ys < height)
            hits = np.zeros_like(inside)
            hits[inside] = edges[ys[inside], xs[inside]] > 0
            support = float(hits.any(axis=1).mean())
            if reason is None and support < p['min_edge_support']:
                reason = 'WEAK_EDGE_SUPPORT'
            score = distance/side + abs(2*r/side-1) + (1-support)
            candidate = {'center': (float(x), float(y)), 'radius': float(r),
                         'support': support, 'score': float(score), 'reason': reason}
            debug['candidates'].append(candidate)
            if reason is None:
                accepted.append(candidate)
        if not accepted:
            return TopCircleResult('NO_RELIABLE_CIRCLE', debug=debug)
        accepted.sort(key=lambda c: c['score'])
        unique = []
        for c in accepted:
            if not any(math.dist(c['center'], u['center']) <= p['duplicate_center_distance_px']
                       and abs(c['radius']-u['radius']) <= p['duplicate_radius_difference_px']
                       for u in unique):
                unique.append(c)
        if len(unique) > 1 and unique[1]['score']-unique[0]['score'] <= p['ambiguity_margin']:
            return TopCircleResult('AMBIGUOUS_CIRCLES', debug=debug)
        best = unique[0]
        return TopCircleResult('OK', best['center'], best['radius'], best['support'], debug)
