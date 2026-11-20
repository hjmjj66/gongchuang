"""Six-color segmentation, multiple contours and explicit mask-color ambiguity."""

import cv2
import numpy as np

from .material_config import COLORS


def detect_hsv(image, config):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    height, width = image.shape[:2]
    masks, candidates, missing = {}, [], []
    kernel = np.ones((config['hsv.morphology_kernel'],) * 2, np.uint8)
    for color_id, color in enumerate(COLORS, 1):
        ranges = config[f'hsv.{color}_ranges']
        mask = np.zeros((height, width), np.uint8)
        if not ranges:
            missing.append(color_id)
        for i in range(0, len(ranges), 6):
            mask |= cv2.inRange(hsv, np.array(ranges[i:i+3], np.uint8),
                               np.array(ranges[i+3:i+6], np.uint8))
        for operation, name in ((cv2.MORPH_OPEN, 'open'), (cv2.MORPH_CLOSE, 'close')):
            iterations = config[f'hsv.{name}_iterations']
            if iterations:
                mask = cv2.morphologyEx(mask, operation, kernel, iterations=iterations)
        masks[color] = mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = float(cv2.contourArea(contour))
            maximum = config['hsv.max_contour_area_ratio']
            if area < config['hsv.min_contour_area_px2'] or (maximum and area > maximum*width*height):
                continue
            moments = cv2.moments(contour)
            if moments['m00'] == 0:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            # Restrict comparison to this contour's real foreground, preserving holes.
            local = np.zeros((h, w), np.uint8)
            cv2.drawContours(local, [contour - (x, y)], -1, 255, cv2.FILLED)
            local &= mask[y:y+h, x:x+w]
            candidates.append({
                'color_id': color_id, 'color_name': color, 'bbox_xyxy': (x,y,x+w,y+h),
                'centroid': (moments['m10']/moments['m00'], moments['m01']/moments['m00']),
                'contour_area_px2': area, 'contour': contour, 'foreground': local,
                'conflicting_colors': [], 'color_overlap_ratio': None,
            })
    for i, a in enumerate(candidates):
        for b in candidates[i+1:]:
            if a['color_id'] == b['color_id']:
                continue
            ax, ay, ar, ab = a['bbox_xyxy']
            bx, by, br, bb = b['bbox_xyxy']
            x, y, r, bottom = max(ax,bx), max(ay,by), min(ar,br), min(ab,bb)
            if r <= x or bottom <= y:
                continue
            common = np.count_nonzero(
                (a['foreground'][y-ay:bottom-ay, x-ax:r-ax] > 0) &
                (b['foreground'][y-by:bottom-by, x-bx:r-bx] > 0))
            denominator = min(np.count_nonzero(a['foreground']), np.count_nonzero(b['foreground']))
            overlap = common / denominator if denominator else 0
            if overlap >= config['hsv.color_overlap_threshold']:
                for first, other in ((a,b),(b,a)):
                    first['conflicting_colors'].append(other['color_id'])
                    first['color_overlap_ratio'] = max(first['color_overlap_ratio'] or 0, overlap)
    return candidates, {'masks': masks, 'missing_colors': missing}
