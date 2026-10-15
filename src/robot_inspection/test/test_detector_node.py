"""In-process tests of anomaly_detector_node against a fake Ollama server."""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import threading
import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
Parameter = pytest.importorskip('rclpy.parameter').Parameter
Image = pytest.importorskip('sensor_msgs.msg').Image
Trigger = pytest.importorskip('std_srvs.srv').Trigger
AnomalyDetectorNode = pytest.importorskip(
    'robot_inspection.anomaly_detector_node'
).AnomalyDetectorNode

# model -> capabilities; llama3 cannot see images
INSTALLED = {
    'qwen3-vl:2b-instruct': ['completion', 'vision'],
    'moondream:latest': ['completion', 'vision'],
    'llama3:latest': ['completion'],
}


class FakeOllama:
    """Answers /api/tags, /api/show and /api/generate like Ollama; records the requests."""

    def __init__(self):
        self.requests = []
        fake = self

        class Handler(BaseHTTPRequestHandler):
            """Ollama's HTTP API, just enough of it."""

            def log_message(self, *args):
                pass

            def reply(self, code: int, body: dict):
                data = json.dumps(body).encode()
                self.send_response(code)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def do_GET(self):
                self.reply(200, {'models': [{'name': name} for name in INSTALLED]})

            def do_POST(self):
                body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
                fake.requests.append((self.path, body))
                model = body.get('model', '')
                if model not in INSTALLED:
                    self.reply(404, {'error': f"model '{model}' not found"})
                elif self.path == '/api/show':
                    self.reply(200, {'capabilities': INSTALLED[model]})
                elif body.get('images'):
                    self.reply(200, {'response': json.dumps({'type': 'trash'})})
                else:
                    self.reply(200, {'done': True})

        self.server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        self.url = f'http://127.0.0.1:{self.server.server_port}'
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def loads(self) -> list:
        """Return (model, keep_alive) of every load / unload request."""
        return [
            (body['model'], body.get('keep_alive'))
            for path, body in self.requests
            if path == '/api/generate' and not body.get('images')
        ]

    def close(self):
        """Stop the server."""
        self.server.shutdown()
        self.server.server_close()


@pytest.fixture
def ollama():
    """Provide a fake Ollama server."""
    server = FakeOllama()
    yield server
    server.close()


@pytest.fixture
def state_file(tmp_path):
    """Provide where the detector keeps the live choice."""
    return tmp_path / 'inspection.json'


@pytest.fixture
def detector(ollama, state_file):
    """Provide a detector inspecting only at stop points, talking to the fake Ollama."""
    rclpy.init(
        args=[
            '--ros-args',
            '-p', f'ollama_url:={ollama.url}',
            '-p', 'interval_sec:=-1.0',
            '-p', 'image_topic:=/test/image',
            '-p', f'state_file:={state_file}',
        ]
    )
    node = AnomalyDetectorNode()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def wait_for(condition, timeout: float = 5.0) -> bool:
    """Poll condition until it holds or timeout passes."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if condition():
            return True
        time.sleep(0.05)
    return condition()


def frame() -> Image:
    """Return a small bgr8 camera frame."""
    msg = Image(height=48, width=64, encoding='bgr8', step=64 * 3)
    msg.data = np.full((48, 64, 3), 128, np.uint8).tobytes()
    return msg


def test_model_is_loaded_and_switched_with_the_old_one_unloaded(detector, ollama):
    """The chosen model is loaded at start; another one replaces it and frees its memory."""
    assert wait_for(lambda: detector.status()['state'] == 'running')
    assert ollama.loads()[0] == ('qwen3-vl:2b-instruct', -1)

    assert detector.set_parameters([Parameter('model', value='moondream:latest')])[0].successful
    assert wait_for(lambda: ('moondream:latest', -1) in ollama.loads())
    assert ('qwen3-vl:2b-instruct', 0) in ollama.loads()  # unloaded
    assert wait_for(lambda: detector.status()['state'] == 'running')
    assert detector.status()['model'] == 'moondream:latest'


def test_offered_models_are_the_configured_and_installed_vision_models(detector):
    """The handheld can pick the configured models and other installed vision models."""

    def options():
        return {m['name']: m for m in detector.status()['models']}

    assert wait_for(lambda: options()['qwen3-vl:2b-instruct']['installed'] is True)
    options = options()
    assert list(options)[:3] == [
        'qwen3-vl:2b-instruct',
        'moondream:latest',
        'ahmadwaqar/smolvlm2-2.2b-instruct:latest',
    ]
    assert options['qwen3-vl:2b-instruct'] == {
        'name': 'qwen3-vl:2b-instruct', 'installed': True, 'active': True,
    }
    assert options['ahmadwaqar/smolvlm2-2.2b-instruct:latest']['installed'] is False
    assert 'llama3:latest' not in options  # no vision


def test_unknown_model_reports_the_pull_command(detector):
    """A model Ollama does not have puts the detector in error with the fix."""
    assert wait_for(lambda: detector.status()['state'] == 'running')
    detector.set_parameters([Parameter('model', value='nope:1b')])
    assert wait_for(lambda: detector.status()['state'] == 'error')
    assert 'ollama pull nope:1b' in detector.status()['error']


def test_inspection_reports_and_can_be_switched_off(detector):
    """A stop-point check reports an alarm category; switched off it answers at once."""
    assert wait_for(lambda: detector.status()['state'] == 'running')
    detector.on_image(frame())
    response = detector.check_now(Trigger.Request(), Trigger.Response())
    assert response.success and response.message.startswith('trash')
    status = detector.status()
    assert status['last']['category'] == 'trash' and status['last']['reported']
    assert status['reports'] == 1

    detector.set_parameters([Parameter('enabled', value=False)])
    assert wait_for(lambda: detector.status()['state'] == 'off')
    response = detector.check_now(Trigger.Request(), Trigger.Response())
    assert response.success and response.message == 'inspection off'


def test_only_live_parameters_change_while_running(detector):
    """Parameters read at start are refused with the reason; bad values too."""
    result = detector.set_parameters([Parameter('categories', value=['trash'])])[0]
    assert not result.successful and 'restart' in result.reason
    assert not detector.set_parameters([Parameter('model', value=' ')])[0].successful
    assert detector.set_parameters([Parameter('interval_sec', value=5.0)])[0].successful
    assert detector.status()['interval_sec'] == 5.0


def test_live_choice_survives_a_restart(detector, state_file):
    """Model, interval and on/off set live come back after the node restarts."""
    assert detector.set_parameters(
        [
            Parameter('model', value='moondream:latest'),
            Parameter('interval_sec', value=5.0),
            Parameter('enabled', value=False),
        ]
    )[0].successful
    assert json.loads(state_file.read_text()) == {
        'enabled': False, 'model': 'moondream:latest', 'interval_sec': 5.0,
    }
    again = AnomalyDetectorNode()
    try:
        status = again.status()
        assert (status['model'], status['interval_sec'], status['enabled']) == (
            'moondream:latest', 5.0, False,
        )
        assert again.get_parameter('model').value == 'moondream:latest'
    finally:
        again.destroy_node()


def test_off_frees_the_model_and_on_loads_it_again(detector, ollama):
    """Switched off the model leaves Ollama (Jetson RAM is the GPU's); on brings it back."""
    assert wait_for(lambda: detector.status()['state'] == 'running')
    first = next(body for path, body in ollama.requests if path == '/api/generate')
    assert first['options'] == {'num_ctx': 2048}  # as inspected: Ollama does not load it twice

    detector.set_parameters([Parameter('enabled', value=False)])
    assert wait_for(lambda: ('qwen3-vl:2b-instruct', 0) in ollama.loads())
    assert wait_for(lambda: detector.status()['state'] == 'off')

    detector.set_parameters([Parameter('enabled', value=True)])
    assert wait_for(lambda: ollama.loads().count(('qwen3-vl:2b-instruct', -1)) == 2)
    assert wait_for(lambda: detector.status()['state'] == 'running')


def test_a_stopped_node_frees_its_model(detector, ollama):
    """keep_alive -1 holds the model while the node runs, not after it stopped."""
    assert wait_for(lambda: detector.status()['state'] == 'running')
    assert ('qwen3-vl:2b-instruct', 0) not in ollama.loads()
    other = AnomalyDetectorNode()
    assert wait_for(lambda: other.status()['state'] == 'running')
    other.destroy_node()
    assert ('qwen3-vl:2b-instruct', 0) in ollama.loads()
