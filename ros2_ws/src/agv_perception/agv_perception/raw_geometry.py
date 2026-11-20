"""Circle estimates in one stationary camera view (or three simultaneous centers)."""

import math
import numpy as np


def fit_circle(points, config):
    if len(points) < 3:
        return None
    points = np.asarray(points, dtype=float)
    mean = points.mean(axis=0)
    centered = points-mean
    eigen = np.linalg.eigvalsh(centered.T@centered/len(points))
    if (eigen[1] <= 0 or eigen[0]/eigen[1] < config['min_axis_ratio'] or
            np.linalg.norm(np.ptp(points, axis=0)) < config['min_span_px']):
        return None
    solution, _, _, _ = np.linalg.lstsq(
        np.column_stack((2*centered, np.ones(len(points)))), (centered**2).sum(axis=1), rcond=None)
    center = solution[:2]+mean
    radius = math.sqrt(max(0., solution[2]+sum(solution[:2]**2)))
    residual = np.sqrt(np.mean((np.linalg.norm(points-center, axis=1)-radius)**2))
    if not config['min_radius_px'] <= radius <= config['max_radius_px'] or residual > config['max_residual_px']:
        return None
    return (float(center[0]), float(center[1])), radius
