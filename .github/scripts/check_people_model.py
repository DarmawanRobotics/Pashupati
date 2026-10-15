"""Run the people model on a test image with each backend; results become CI annotations."""

import sys
import time

import cv2

sys.path.insert(0, 'src/robot_perception')
from robot_perception.detector import OnnxPersonDetector  # noqa: E402


def main(model: str, image: str, backends: list[str]) -> int:
    """Return 1 when ONNX Runtime does not find the people in the image."""
    bgr = cv2.imread(image)
    if bgr is None:
        print(f'::error::cannot read {image}')
        return 1
    failed = False
    for backend in backends:
        title = f'{backend}, OpenCV {cv2.__version__}'
        try:
            detector = OnnxPersonDetector(model, 0.4, 416, 2, backend)
            detector.detect(bgr)  # warm up
            start = time.monotonic()
            boxes = detector.detect(bgr)
            ms = (time.monotonic() - start) * 1000
        except Exception as error:  # noqa: B902  report what this backend cannot do
            print(f'::warning title={title}::{type(error).__name__}: {error}')
            failed |= backend == 'onnxruntime'
            continue
        found = ', '.join(f'{b[4]:.2f}' for b in boxes)
        print(f'::notice title={title}::{len(boxes)} people ({found}) in {ms:.0f} ms')
        if len(boxes) < 3:
            print(f'::warning title={title}::expected the 4 people of the test image')
            failed |= backend == 'onnxruntime'
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1], sys.argv[2], sys.argv[3].split(',')))
