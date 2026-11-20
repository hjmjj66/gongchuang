"""Shared B/C/F overlays and diagnostic image/JSON output."""

import json
from pathlib import Path

import cv2


def draw_result(frame, result):
    canvas = frame.copy()
    for contour in result.debug.get('contours', []):
        cv2.drawContours(canvas, [contour], -1, (255,180,80), 1)
    for index, item in enumerate(result.observations, 1):
        if item.color_bbox_xyxy:
            a,b,c,d = (round(v) for v in item.color_bbox_xyxy)
            cv2.rectangle(canvas,(a,b),(c,d),(255,180,80),1)
        if item.material_bbox_xyxy:
            a,b,c,d = (round(v) for v in item.material_bbox_xyxy)
            cv2.rectangle(canvas,(a,b),(c,d),(160,210,100),1)
        x1,y1,x2,y2 = (round(v) for v in item.bbox_xyxy)
        cv2.rectangle(canvas, (x1,y1), (x2,y2), (70,210,70) if item.valid else (90,90,230), 2)
        text = f'{index}: {item.color_id} {item.color_name} {item.status}'
        if item.color_reason and item.color_reason != 'OK':
            text += f' {item.color_reason}'
        cv2.putText(canvas, text, (max(0,x1),max(16,y1-8)), cv2.FONT_HERSHEY_SIMPLEX, .45, (255,255,255), 1)
        if item.roi_xyxy:
            a,b,c,d = item.roi_xyxy
            cv2.rectangle(canvas, (a,b), (c,d), (150,150,150), 1)
        cv2.drawMarker(canvas, tuple(round(v) for v in item.bbox_center_px),
                       (255,255,0), cv2.MARKER_CROSS, 18, 2)
        if item.circle_center_px:
            point = tuple(round(v) for v in item.circle_center_px)
            if item.ellipses_px:
                for ellipse in item.ellipses_px:
                    cv2.ellipse(canvas,ellipse,(255,0,255),1)
            else:
                cv2.circle(canvas, point, round(item.radius_px), (255,0,255), 1)
            cv2.circle(canvas, point, 3, (255,0,255), -1)
        if item.center_px:
            cv2.drawMarker(canvas, tuple(round(v) for v in item.center_px),
                           (0,255,255), cv2.MARKER_DIAMOND, 12, 2)
    return canvas


def save_result(directory, frame, result):
    directory = Path(directory).expanduser()
    directory.mkdir(parents=True, exist_ok=True)
    (directory / 'result.json').write_text(
        json.dumps(result.to_dict(), ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    _write_image(directory / 'annotated.jpg', draw_result(frame, result))
    for color, mask in result.debug.get('masks', {}).items():
        _write_image(directory / f'mask_{color}.png', mask)
    for i, geometry in enumerate(result.debug.get('geometry', [])):
        for name in ('gray', 'edges'):
            _write_image(directory / f'top_{i}_{name}.png', geometry[name])
        candidate_image = cv2.cvtColor(geometry['gray'], cv2.COLOR_GRAY2BGR)
        for candidate in geometry['candidates']:
            cv2.circle(candidate_image, tuple(round(v) for v in candidate['center']),
                       round(candidate['radius']), (0,0,255) if candidate['reason'] else (0,255,0), 1)
        _write_image(directory / f'top_{i}_candidates.png', candidate_image)


def _write_image(path, frame):
    if not cv2.imwrite(str(path), frame):
        raise OSError(f'无法保存图片：{path}')
