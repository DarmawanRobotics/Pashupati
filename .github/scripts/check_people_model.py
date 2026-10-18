"""Run the people model on a test image with each backend; results become CI annotations."""

import sys
import time
import traceback


def annotate(level: str, title: str, text: str):
    """Print a GitHub annotation (commas end a title, newlines need escaping)."""
    text = text.replace('%', '%25').replace('\r', '%0D').replace('\n', '%0A')
    print(f'::{level} title={title.replace(",", " ")}::{text}', flush=True)


def main(model: str, image: str, backends: list[str]) -> int:
    """Return 1 when ONNX Runtime does not find the people in the image."""
    try:
        import cv2

        sys.path.insert(0, 'src/pashupati_perception')
        from pashupati_perception.detector import OnnxPersonDetector
    except Exception:  # noqa: B902  the environment itself is broken: say how
        annotate('error', 'imports', traceback.format_exc()[-1500:])
        return 1
    bgr = cv2.imread(image)
    if bgr is None:
        annotate('error', 'image', f'cannot read {image}')
        return 1
    failed = False
    for backend in backends:
        title = f'{backend} on OpenCV {cv2.__version__}'
        try:
            detector = OnnxPersonDetector(model, 0.4, 416, 2, backend)
            detector.detect(bgr)  # warm up
            start = time.monotonic()
            boxes = detector.detect(bgr)
            ms = (time.monotonic() - start) * 1000
        except Exception as error:  # noqa: B902  report what this backend cannot do
            annotate('warning', title, f'{type(error).__name__}: {error}')
            failed |= backend == 'onnxruntime'
            continue
        found = ' '.join(f'{b[4]:.2f}' for b in boxes)
        annotate('notice', title, f'{len(boxes)} people ({found}) in {ms:.0f} ms')
        if len(boxes) < 3:
            annotate('warning', title, 'expected the 4 people of the test image')
            failed |= backend == 'onnxruntime'
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1], sys.argv[2], sys.argv[3].split(',')))
