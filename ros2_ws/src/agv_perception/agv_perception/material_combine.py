"""Same-frame YOLO color identity and JHB top-center fusion."""

import time

from .material_jhb import JhbMaterialModel
from .material_model import load_material_model


class CombinedMaterialModel:
    def __init__(self, config):
        self.position = JhbMaterialModel(config, load_color=False)
        self.geometry = self.position.geometry
        self.color = load_material_model({**config, 'test_combine': False,
                                          'use_jhb_model': False, 'material_backend': 2})

    def infer(self, image_bgr):
        started = time.perf_counter()
        positions = self.position.infer(image_bgr)
        jhb_ms = (time.perf_counter()-started)*1000
        started = time.perf_counter()
        colors = self.color.infer(image_bgr)
        color_ms = (time.perf_counter()-started)*1000
        return positions, colors, dict(jhb_yolo=jhb_ms, color_yolo=color_ms,
                                      yolo=jhb_ms+color_ms)


def match_colors(pairs, colors, config):
    """Accept only unique pairings in both directions; never resolve overlaps by color."""
    colors = [c for c in colors if c.confidence > config['confidence_threshold']]
    links = []
    for cap, material in pairs:
        box = cap.bbox_xyxy if config['material_model'] == 1 else material.bbox_xyxy
        x1,y1,x2,y2 = box
        cx,cy = ((cap.bbox_xyxy[i]+cap.bbox_xyxy[i+2])/2 for i in (0,1))
        candidates = []
        for index, color in enumerate(colors):
            a,b,c,d = color.bbox_xyxy
            intersection = max(0,min(x2,c)-max(x1,a))*max(0,min(y2,d)-max(y1,b))
            union = (x2-x1)*(y2-y1)+(c-a)*(d-b)-intersection
            if (intersection/union >= config['combine.min_iou'] and
                    a <= cx <= c and b <= cy <= d):
                candidates.append(index)
        links.append(candidates)
    uses = [0]*len(colors)
    for candidates in links:
        for index in candidates:
            uses[index] += 1
    return [(colors[candidates[0]], 'OK') if len(candidates) == 1 and uses[candidates[0]] == 1
            else (None, 'COLOR_MATCH_AMBIGUOUS' if candidates else 'COLOR_UNMATCHED')
            for candidates in links]
