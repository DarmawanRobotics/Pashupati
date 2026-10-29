"""Camera frames as BGR arrays, and the JPEGs made of them (OpenCV, no ROS)."""

from __future__ import annotations

import numpy as np

# encoding -> (channels, name of the cv2 conversion to BGR or None)
ENCODINGS = {
    'rgb8': (3, 'COLOR_RGB2BGR'),
    'bgr8': (3, None),
    'rgba8': (4, 'COLOR_RGBA2BGR'),
    'bgra8': (4, 'COLOR_BGRA2BGR'),
    'mono8': (1, 'COLOR_GRAY2BGR'),
}


def to_bgr(encoding: str, height: int, width: int, step: int, data) -> np.ndarray | None:
    """Return a BGR copy of a raw image, None for an unsupported encoding or size."""
    import cv2

    spec = ENCODINGS.get(encoding.lower())
    if spec is None or height <= 0 or width <= 0 or step < width * spec[0]:
        return None
    channels, conversion = spec
    buffer = np.frombuffer(data, np.uint8)
    if buffer.size < height * step:
        return None
    rows = buffer[: height * step].reshape(height, step)
    img = rows[:, : width * channels].reshape(height, width, channels)
    img = img[:, :, 0] if channels == 1 else img
    return cv2.cvtColor(img, getattr(cv2, conversion)) if conversion else img.copy()


def decode_jpeg(data: bytes) -> np.ndarray | None:
    """Return the BGR image of a JPEG (or PNG), None when it does not decode."""
    import cv2

    return cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)


def fit_width(img: np.ndarray, max_width: int) -> np.ndarray:
    """Return img scaled down to max_width (same aspect ratio), or img itself."""
    import cv2

    height, width = img.shape[:2]
    if width <= max_width:
        return img
    size = (max_width, max(1, int(height * max_width / width)))
    return cv2.resize(img, size, interpolation=cv2.INTER_AREA)


def jpeg(img: np.ndarray, quality: int = 90) -> bytes:
    """Return img as JPEG bytes."""
    import cv2

    ok, buffer = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    return buffer.tobytes() if ok else b''
