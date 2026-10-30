"""anomaly_detector_node: early anomaly detection with a local vision-language model (Ollama)."""

from __future__ import annotations

import json
import threading
import time

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.parameter import Parameter
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger

from dharma_utils.ros import EVENTS, SENSOR, DharmaNode, run
from pashupati_inspection.core.choice import CATEGORY_KEYS, read_choice, write_choice
from pashupati_inspection.core.config import AnomalyConfig
from pashupati_inspection.core.engine import STOP_POINT, InspectionEngine
from pashupati_inspection.core.frame_queue import FrameQueue
from pashupati_inspection.core.images import to_bgr
from pashupati_inspection.core.loop import Frame, InspectionLoop

LIVE_KEYS = ('enabled', 'model', 'interval_sec', *CATEGORY_KEYS)
STATUS_PERIOD_SEC = 2.0  # anomaly_detector/status also when nothing changed
MODELS_PERIOD_SEC = 30.0  # how often Ollama is asked which models it has
CAPTURE_TICK_SEC = 0.05  # interval mode: how often a frame is checked for being due


def evidence_stamp(sec: int, nanosec: int) -> str:
    """Return a camera stamp as 'sec.nanosec', the key linking a result to its evidence frame."""
    return f'{sec}.{nanosec:09d}'


class AnomalyDetectorNode(DharmaNode):
    """Inspects the floor at stop points (follower hook) and, optionally, all the time."""

    def __init__(self, **kwargs):
        super().__init__('anomaly_detector_node', **kwargs)
        self.config = self.declare_config(AnomalyConfig)
        self.own_categories = False  # finding types changed live: kept in state_file
        self.closing = False  # set first in destroy_node: the threads stop publishing
        self.frame_lock = threading.Lock()
        self.frame = (0.0, None)  # (receive time, Image)
        self.engine = InspectionEngine(self.config.value)
        self.queue = FrameQueue(self.config.queue_dir, self.config.queue_max)
        self.loop = InspectionLoop(
            self.engine, self.queue, self.latest_frame, self.publish_verdict, self.write_log
        )
        self.result_pub = self.create_publisher(String, 'anomaly_detector/result', EVENTS)
        self.evidence_pub = self.create_publisher(
            CompressedImage, 'anomaly_detector/evidence', EVENTS
        )
        self.status_pub = self.latched(String, 'anomaly_detector/status')
        self.create_subscription(
            Image,
            self.config.image_topic,
            self.on_image,
            SENSOR,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.create_service(
            Trigger, '~/check_now', self.check_now, callback_group=ReentrantCallbackGroup()
        )
        self.config.on_change(self.on_config)
        self.restore_choice()
        self.every(STATUS_PERIOD_SEC, self.publish_status)
        self.every(CAPTURE_TICK_SEC, self.capture, group=MutuallyExclusiveCallbackGroup())
        self.every(MODELS_PERIOD_SEC, self.refresh_models, group=MutuallyExclusiveCallbackGroup())
        cfg = self.config.value
        self.log.info(
            'anomaly detector',
            model=cfg.model,
            enabled=cfg.enabled,
            categories=','.join(cfg.categories),
            alarm=','.join(cfg.alarm_categories),
            interval_sec=cfg.interval_sec,
        )
        self.publish_status()
        self.worker = threading.Thread(target=self.loop.run, daemon=True)
        self.worker.start()
        threading.Thread(target=self.refresh_models, daemon=True).start()

    # ---------------------------------------------------------------- live choices
    def on_config(self, old: AnomalyConfig, new: AnomalyConfig):
        """Apply a live change: model, pace, on / off, finding types; keep it for a restart."""
        self.engine.config = new
        if new.interval_sec != old.interval_sec:
            self.loop.next_capture = 0.0  # the first frame at once
        if any(getattr(new, k) != getattr(old, k) for k in CATEGORY_KEYS):
            self.own_categories = True
            self.log.info('finding types', types=','.join(new.categories),
                          reported=','.join(new.alarm_categories))  # fmt: skip
        self.log.info('inspection', on=new.enabled, model=new.model, interval=new.interval_sec)
        self.save_choice(new)
        self.loop.wake.set()
        self.publish_status()

    def restore_choice(self):
        """Apply what was chosen live before a restart (state_file wins over the files)."""
        values, error = read_choice(self.config.state_file, LIVE_KEYS)
        if error:
            self.log.warning('ignoring the saved inspection choice', error=error)
        if not values:
            return
        results = self.set_parameters([Parameter(k, value=v) for k, v in values.items()])
        refused = [r.reason for r in results if not r.successful]
        if refused:
            self.log.warning('saved inspection choice refused', reason='; '.join(refused))
            return
        self.own_categories = any(k in values for k in CATEGORY_KEYS)
        self.log.info('live choice restored', path=self.config.state_file, keys=','.join(values))

    def save_choice(self, config: AnomalyConfig):
        """Keep the live choice for the next start."""
        choice = {
            'enabled': config.enabled,
            'model': config.model,
            'interval_sec': config.interval_sec,
        }
        if self.own_categories:  # else the parameter files keep deciding them
            choice.update({k: list(getattr(config, k)) for k in CATEGORY_KEYS})
        error = write_choice(self.config.state_file, choice)
        if error:
            self.log.warning('cannot keep the inspection choice', error=error)

    # ---------------------------------------------------------------- frames
    def on_image(self, msg: Image):
        """Keep the latest camera frame; it is converted only when inspected."""
        with self.frame_lock:
            self.frame = (time.monotonic(), msg)

    def latest_frame(self) -> Frame | None:
        """Return the latest frame when it is fresh and in a supported encoding."""
        with self.frame_lock:
            received, msg = self.frame
        if msg is None or time.monotonic() - received > self.config.max_image_age_sec:
            return None
        image = to_bgr(msg.encoding, msg.height, msg.width, msg.step, msg.data)
        if image is None:
            self.log.error('unsupported camera image', encoding=msg.encoding, throttle=30.0)
            return None
        stamp = msg.header.stamp
        return Frame(image, stamp.sec, stamp.nanosec, msg.header.frame_id, time.time())

    def capture(self):
        """Interval mode: queue a frame when one is due."""
        self.loop.capture()

    def refresh_models(self):
        """Ask Ollama for its models (the handheld offers the vision ones)."""
        if self.engine.refresh_models():
            self.publish_status()

    # ---------------------------------------------------------------- results
    def publish_verdict(self, verdict, frame: Frame, evidence: bytes):
        """Publish the analysed frame, then the result that names it."""
        header = Header(frame_id=frame.frame_id)
        header.stamp.sec, header.stamp.nanosec = frame.sec, frame.nanosec
        result = dict(verdict.result)
        result['evidence_stamp'] = evidence_stamp(frame.sec, frame.nanosec)
        self.evidence_pub.publish(CompressedImage(header=header, format='jpeg', data=evidence))
        self.result_pub.publish(String(data=json.dumps(result)))
        self.log.info('finding reported', source=result['source'], detail=verdict.message)

    def check_now(self, request, response):
        """Service: inspect at a stop point (the follower's hooks.inspection)."""
        if not self.config.enabled:
            response.success, response.message = True, 'inspection off'
            return response
        verdict, message = self.loop.inspect(STOP_POINT, wait=self.config.timeout_sec)
        self.after_inspection()
        response.success, response.message = verdict is not None, message
        self.log.info('check_now', detail=message)
        return response

    def after_inspection(self):
        """Log a debug answer and publish the status after an inspection."""
        answer, valid = self.engine.last_answer
        if not valid:
            self.log.warning('no known type in the answer', answer=answer[:200])
        elif self.config.debug:
            self.log.info('model answer', answer=answer[:300])
        self.publish_status()

    def status(self) -> dict:
        """Return the status JSON of anomaly_detector/status."""
        return self.engine.status(self.queue)

    def publish_status(self):
        """Publish the status (latched): on every change and every STATUS_PERIOD_SEC."""
        if self.closing:
            return
        try:
            self.status_pub.publish(String(data=json.dumps(self.status())))
        except Exception:  # destroyed under a worker thread while stopping
            if not self.closing:
                raise

    def write_log(self, level: str, message: str):
        """Log for the worker loop (throttled: it retries every few seconds)."""
        getattr(self.log, level)(message, throttle=10.0)
        self.publish_status()

    def destroy_node(self):
        """Stop the worker and free the model: a stopped stack must not hold the robot's RAM."""
        self.closing = True
        self.loop.stop.set()
        self.loop.wake.set()
        self.worker.join(timeout=1.0)
        model = self.engine.unload()
        if model:
            self.log.info('model unloaded', model=model)
        return super().destroy_node()


def main(args=None):
    """Run the anomaly detector (three threads: images, the service, timers)."""
    run(AnomalyDetectorNode, threads=3, args=args)


if __name__ == '__main__':
    main()
