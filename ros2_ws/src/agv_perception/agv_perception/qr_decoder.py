"""Contour-based QR decoding, with geometry mapped back to the input image."""

from dataclasses import dataclass

import cv2
import numpy as np
from pyzbar.pyzbar import ZBarSymbol, decode as zbar_decode


@dataclass
class QRDetection:
    text: str
    center: tuple[float, float]
    polygon: np.ndarray
    area: float


def _decode_text(data: bytes) -> str:
    for encoding in ('utf-8', 'gbk'):
        try:
            return data.decode(encoding)
        except UnicodeDecodeError:
            pass
    return data.decode('utf-8', errors='replace')


def _order_quad(points: np.ndarray) -> np.ndarray:
    points = points.astype(np.float32)
    ordered = np.empty((4, 2), dtype=np.float32)
    sums = points.sum(axis=1)
    diffs = np.diff(points, axis=1).ravel()
    ordered[0] = points[np.argmin(sums)]
    ordered[2] = points[np.argmax(sums)]
    ordered[1] = points[np.argmin(diffs)]
    ordered[3] = points[np.argmax(diffs)]
    # Sum/difference ties at 45 degrees can select the same vertex twice.
    if len(np.unique(ordered, axis=0)) != 4:
        middle = points.mean(axis=0)
        angles = np.arctan2(points[:, 1] - middle[1], points[:, 0] - middle[0])
        ordered = points[np.argsort(angles)]
        ordered = np.roll(ordered, -np.argmin(ordered.sum(axis=1)), axis=0)
    return ordered


def _quad_center(polygon: np.ndarray) -> np.ndarray:
    """Find the diagonal intersection, which preserves the QR center in perspective."""
    a, b, c, d = _order_quad(polygon)
    matrix = np.column_stack((c - a, b - d))
    fraction = np.linalg.solve(matrix, d - a)[0]
    return a + fraction * (c - a)


class PreprocessedQRDecoder:
    def __init__(self):
        self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (21, 21))
        self.destination = np.float32([[0, 0], [499, 0], [499, 499], [0, 499]])

    def decode(self, frame: np.ndarray) -> QRDetection | None:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        closed = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, self.kernel)
        contours, _ = cv2.findContours(closed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        min_area = frame.shape[0] * frame.shape[1] * 0.01
        best = None

        for contour in sorted(contours, key=cv2.contourArea, reverse=True):
            if cv2.contourArea(contour) < min_area:
                break
            hull = cv2.convexHull(contour)
            perimeter = cv2.arcLength(hull, True)
            quad = cv2.approxPolyDP(hull, 0.02 * perimeter, True)
            if len(quad) != 4:
                continue
            source = _order_quad(quad.reshape(4, 2))
            if abs(cv2.contourArea(source)) < 1.0:
                continue
            transform = cv2.getPerspectiveTransform(source, self.destination)
            corrected = cv2.warpPerspective(gray, transform, (500, 500))
            results = zbar_decode(corrected, symbols=[ZBarSymbol.QRCODE])
            if not results:
                continue
            inverse = np.linalg.inv(transform)
            for barcode in results:
                # Use the decoded QR boundary, not a surrounding board/contour.
                polygon = np.float32([(p.x, p.y) for p in barcode.polygon])
                polygon = cv2.convexHull(polygon).reshape(-1, 2)
                if len(polygon) != 4:
                    x, y, width, height = barcode.rect
                    polygon = np.float32([
                        [x, y], [x + width, y],
                        [x + width, y + height], [x, y + height],
                    ])
                try:
                    center = _quad_center(polygon)
                except np.linalg.LinAlgError:
                    continue
                mapped = cv2.perspectiveTransform(polygon.reshape(1, -1, 2), inverse)[0]
                mapped_center = cv2.perspectiveTransform(center.reshape(1, 1, 2), inverse)[0, 0]
                area = abs(cv2.contourArea(mapped))
                if best is None or area > best.area:
                    best = QRDetection(
                        text=_decode_text(barcode.data),
                        center=(float(mapped_center[0]), float(mapped_center[1])),
                        polygon=mapped,
                        area=area,
                    )
        return best
