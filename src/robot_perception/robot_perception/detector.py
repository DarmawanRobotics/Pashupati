import numpy as np


class PersonDetector:
    """Ultralytics YOLO person detector (.pt, .onnx or a TensorRT .engine)."""

    PERSON = 0

    def __init__(self, model_path: str, confidence: float = 0.4, image_size: int = 640):
        from ultralytics import YOLO

        self._model = YOLO(model_path, task='detect')
        self._confidence = confidence
        self._image_size = image_size

    def detect(self, bgr: np.ndarray) -> list[tuple]:
        """Return person boxes as (x1, y1, x2, y2, score) in pixels."""
        result = self._model.predict(
            bgr,
            conf=self._confidence,
            imgsz=self._image_size,
            classes=[self.PERSON],
            verbose=False,
        )[0]
        boxes = result.boxes
        if boxes is None or len(boxes) == 0:
            return []
        xyxy = boxes.xyxy.cpu().numpy()
        scores = boxes.conf.cpu().numpy()
        return [(*map(float, b), float(s)) for b, s in zip(xyxy, scores)]
