"""Shared original-image overlays for ROS and browser debugging."""

import cv2


BBOX_CENTER_COLOR = (255, 220, 0)   # BGR: cyan
CIRCLE_CENTER_COLOR = (255, 0, 255)


def draw_result(frame, result):
    preview = frame.copy()
    for item in result.observations:
        if item.bbox_xyxy is None:
            continue
        x1, y1, x2, y2 = (round(v) for v in item.bbox_xyxy)
        cv2.rectangle(preview, (x1, y1), (x2, y2), (0, 200, 255), 2)
        if item.roi_xyxy is not None:
            rx1, ry1, rx2, ry2 = item.roi_xyxy
            cv2.rectangle(preview, (rx1, ry1), (rx2, ry2), (140, 140, 140), 1)
        label = (f'ring_{item.ring_id}' if item.identity_source == 'yolo'
                 else f'manual:{item.ring_id}')
        if item.yolo_confidence is not None:
            label += f' {item.yolo_confidence:.3f}'
        label += f' {item.status}'
        cv2.putText(preview, label, (max(0, x1), max(18, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2, cv2.LINE_AA)
        bbox_center = tuple(round(v) for v in item.bbox_center_px)
        cv2.drawMarker(preview, bbox_center, BBOX_CENTER_COLOR, cv2.MARKER_CROSS, 22, 2)
        bx, by = item.bbox_center_px
        cv2.putText(preview, f'B ({bx:.1f}, {by:.1f})',
                    (max(0, bbox_center[0] + 8), max(16, bbox_center[1] - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, BBOX_CENTER_COLOR, 1, cv2.LINE_AA)
        if item.circle_center_px is not None:
            center = tuple(round(v) for v in item.circle_center_px)
            # A hollow circle stays visible when the geometric center equals the box center.
            cv2.circle(preview, center, 7, CIRCLE_CENTER_COLOR, 2, cv2.LINE_AA)
            cv2.circle(preview, center, 2, CIRCLE_CENTER_COLOR, -1)
            if item.ellipses_px:
                for ellipse in item.ellipses_px:
                    cv2.ellipse(preview, ellipse, (70, 210, 70), 1, cv2.LINE_AA)
            else:
                for radius in item.radii_px:
                    cv2.circle(preview, center, round(radius), (70, 210, 70), 1, cv2.LINE_AA)
            cx, cy = item.circle_center_px
            cv2.putText(preview, f'C ({cx:.1f}, {cy:.1f})',
                        (max(0, center[0] + 8), min(frame.shape[0] - 5, center[1] + 22)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, CIRCLE_CENTER_COLOR, 1, cv2.LINE_AA)
    return preview
