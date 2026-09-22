#!/usr/bin/env python3
"""ROS1 live semantic inference with selectable MORAI camera views."""

from __future__ import annotations

import json
import threading
import time

import cv2
import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import CompressedImage, Image

from camera_semantic_perception.backends import create_backend
from camera_semantic_perception.frame_buffer import EncodedFrame, LatestFrameBuffer
from camera_semantic_perception.inference_engine import InferenceEngine


SUPPORTED_VIEWS = ("front", "left", "right", "rear")


class SemanticInferenceNode:
    def __init__(self) -> None:
        self.views = tuple(str(value) for value in rospy.get_param("~views", ["front"]))
        unknown = set(self.views) - set(SUPPORTED_VIEWS)
        if not self.views or unknown or len(set(self.views)) != len(self.views):
            raise ValueError(
                "views must be unique values from {}; got {}".format(
                    SUPPORTED_VIEWS, self.views
                )
            )
        camera_topics = dict(rospy.get_param("~camera_topics"))
        missing_topics = [view for view in self.views if view not in camera_topics]
        if missing_topics:
            raise ValueError("camera_topics missing views: {}".format(missing_topics))

        backend_name = str(rospy.get_param("~backend", "twinlite"))
        backend = create_backend(
            backend_name,
            checkpoint=str(rospy.get_param("~checkpoint")),
            model_config=str(rospy.get_param("~model_config", "medium")),
            twinlite_root=rospy.get_param("~twinlite_root", None),
            device=str(rospy.get_param("~device", "cuda")),
            weights=str(rospy.get_param("~weights", "auto")),
            amp=bool(rospy.get_param("~amp", True)),
            channels_last=bool(rospy.get_param("~channels_last", False)),
            input_hw=tuple(
                int(value)
                for value in rospy.get_param("~input_hw", [384, 640])
            ),
        )
        self.engine = InferenceEngine(
            backend,
            restore_original=bool(rospy.get_param("~restore_original", True)),
        )
        warmup_iterations = int(rospy.get_param("~warmup_iterations", 1))
        if warmup_iterations < 0:
            raise ValueError("warmup_iterations must be non-negative")
        if warmup_iterations:
            warmup_image = np.zeros(
                (backend.info.input_hw[0], backend.info.input_hw[1], 3),
                dtype=np.uint8,
            )
            warmup_views = dict((view, warmup_image) for view in self.views)
            started = time.perf_counter()
            for _ in range(warmup_iterations):
                self.engine.infer(warmup_views)
            rospy.loginfo(
                "camera semantic warmup complete: iterations=%d views=%d elapsed_ms=%.1f",
                warmup_iterations,
                len(self.views),
                (time.perf_counter() - started) * 1000.0,
            )
        self.buffer = LatestFrameBuffer(
            self.views,
            batch_wait_ms=float(rospy.get_param("~batch_wait_ms", 4.0)),
        )
        self.log_interval = float(rospy.get_param("~log_interval_sec", 2.0))
        if self.log_interval <= 0:
            raise ValueError("log_interval_sec must be positive")
        self.categorical_heads = frozenset(backend.info.categorical_classes)
        self.backend_info = dict(backend.info.__dict__)
        self.output_topics = {
            view: {
                head: self._output_topic(view, head, head in self.categorical_heads)
                for head in backend.info.heads
            }
            for view in self.views
        }
        self.confidence_topics = {
            view: {
                head: "/perception/camera/{}/{}/confidence".format(view, head)
                for head in self.categorical_heads
            }
            for view in self.views
        }
        self.publishers = {
            view: {
                head: rospy.Publisher(
                    self.output_topics[view][head],
                    Image,
                    queue_size=1,
                )
                for head in backend.info.heads
            }
            for view in self.views
        }
        self.confidence_publishers = {
            view: {
                head: rospy.Publisher(topic, Image, queue_size=1)
                for head, topic in self.confidence_topics[view].items()
            }
            for view in self.views
        }
        self.publish_model_grid_road_marking = bool(
            rospy.get_param("~publish_model_grid_road_marking", False)
        )
        self.model_grid_max_hz = float(rospy.get_param("~model_grid_max_hz", 10.0))
        if self.model_grid_max_hz <= 0.0:
            raise ValueError("model_grid_max_hz must be positive")
        self.model_grid_period_ns = int(round(1e9 / self.model_grid_max_hz))
        self.next_model_grid_stamp_ns = 0
        self.last_model_grid_stamp_ns = 0
        self.model_grid_timestamp_resets = 0
        self.model_grid_publishers = {}
        self.model_grid_confidence_publishers = {}
        if (
            self.publish_model_grid_road_marking
            and "road_marking" in self.categorical_heads
        ):
            for view in self.views:
                namespace = "/perception/camera/{}/road_marking/model_grid".format(
                    view
                )
                self.model_grid_publishers[view] = rospy.Publisher(
                    namespace + "/class_id", Image, queue_size=1
                )
                self.model_grid_confidence_publishers[view] = rospy.Publisher(
                    namespace + "/confidence", Image, queue_size=1
                )
        self.diagnostics_publisher = rospy.Publisher(
            "/perception/camera/diagnostics", DiagnosticArray, queue_size=1
        )
        self.decode_errors = 0
        self.inference_errors = 0
        self.output_frames = 0
        self.output_by_view = {view: 0 for view in self.views}
        self.output_batches = 0
        self.total_output_batches = 0
        self.model_grid_output_batches = 0
        self.model_grid_throttled_batches = 0
        self.model_grid_incomplete_batches = 0
        self.timing_totals = {
            "decode_ms": 0.0,
            "preprocess_ms": 0.0,
            "inference_ms": 0.0,
            "postprocess_ms": 0.0,
            "publish_ms": 0.0,
        }
        self._stats_lock = threading.Lock()
        self._last_log_wall = time.monotonic()
        self._last_output_frames = 0
        self._last_decode_errors = 0
        self._last_inference_errors = 0
        self._queue_age_samples = []
        self._end_to_end_samples = []
        self._latest_input_stamp_ns = {}
        self._latest_receive_stamp_ns = {}
        self._latest_result_stamp_ns = 0
        # MORAI rosbridge 가 header.stamp 를 0 으로 보내면 visualizer/drivable_bev 의
        # exact-time TimeSynchronizer 가 조용히 아무것도 내보내지 않는다. 그 실패를
        # 30 초 안에 찾기 위한 view 별 카운터. (ASMC 에서는 UDP bridge 가 이 정보를
        # /morai_sensor_bridge/diagnostics 로 알려줬지만 VIP3 에는 bridge 가 없다.)
        self._source_stamp_zero_by_view = {view: 0 for view in self.views}

        self.subscribers = []
        for view in self.views:
            subscriber = rospy.Subscriber(
                str(camera_topics[view]),
                CompressedImage,
                self._callback,
                callback_args=view,
                queue_size=1,
                buff_size=int(rospy.get_param("~subscriber_buffer_bytes", 16 << 20)),
            )
            self.subscribers.append(subscriber)
        self.worker = threading.Thread(target=self._worker, name="semantic-inference")
        self.worker.daemon = True
        self.worker.start()
        rospy.on_shutdown(self._shutdown)

        rospy.loginfo(
            "camera semantic backend: %s",
            json.dumps(backend.info.__dict__, sort_keys=True),
        )
        rospy.loginfo(
            "camera semantic views=%s inputs=%s outputs=%s confidences=%s "
            "model_grid_road_marking=%s model_grid_max_hz=%.2f",
            self.views,
            camera_topics,
            self.output_topics,
            self.confidence_topics,
            bool(self.model_grid_publishers),
            self.model_grid_max_hz,
        )

    @staticmethod
    def _output_topic(view: str, head: str, categorical: bool) -> str:
        suffix = "class_id" if categorical else "probability"
        return "/perception/camera/{}/{}/{}".format(view, head, suffix)

    def _callback(self, message: CompressedImage, view: str) -> None:
        if message.header.stamp.is_zero():
            self._source_stamp_zero_by_view[view] += 1
        self.buffer.push(
            EncodedFrame(
                view=view,
                stamp=message.header.stamp,
                frame_id=message.header.frame_id,
                data=bytes(message.data),
                received_monotonic=time.monotonic(),
                received_ros_ns=int(rospy.Time.now().to_nsec()),
            )
        )

    @staticmethod
    def _image_message(
        value: np.ndarray, frame: EncodedFrame, categorical: bool = False
    ) -> Image:
        if categorical:
            value = np.ascontiguousarray(value, dtype=np.uint8)
        else:
            value = np.rint(np.clip(value, 0.0, 1.0) * 255.0).astype(np.uint8)
        message = Image()
        message.header.stamp = frame.stamp
        message.header.frame_id = frame.frame_id
        message.height, message.width = value.shape
        message.encoding = "mono8"
        message.is_bigendian = 0
        message.step = message.width
        message.data = value.tobytes()
        return message

    @staticmethod
    def _publish_unless_shutdown(publisher, message) -> bool:
        """Publish without reporting a false inference failure during shutdown."""
        if rospy.is_shutdown():
            return False
        try:
            publisher.publish(message)
        except rospy.ROSException:
            if rospy.is_shutdown():
                return False
            raise
        return True

    def _worker(self) -> None:
        while not rospy.is_shutdown():
            frames = self.buffer.pop_batch(timeout=0.2)
            if not frames:
                continue
            decoded = {}
            usable_frames = {}
            started = time.perf_counter()
            for frame in frames:
                image = cv2.imdecode(
                    np.frombuffer(frame.data, dtype=np.uint8), cv2.IMREAD_COLOR
                )
                if image is None:
                    self.decode_errors += 1
                    rospy.logwarn_throttle(2.0, "failed to decode camera JPEG")
                    continue
                decoded[frame.view] = image
                usable_frames[frame.view] = frame
            decode_ms = (time.perf_counter() - started) * 1000.0
            if not decoded:
                continue
            try:
                # Snapshot the lazy-publish decision for the complete batch.
                # Subscriber connections may change while GPU inference is in
                # progress; re-checking afterward could request arrays that
                # were intentionally not copied into this prediction.
                preserve_model_grid = self._model_grid_has_subscribers()
                prediction = self.engine.infer(
                    decoded,
                    preserve_model_grid=preserve_model_grid,
                )
            except Exception as exc:
                with self._stats_lock:
                    self.inference_errors += 1
                rospy.logerr_throttle(2.0, "semantic inference failed: %s", exc)
                continue

            started = time.perf_counter()
            publish_model_grid = preserve_model_grid and self._should_publish_model_grid(
                prediction.views, usable_frames
            )
            for view, value in prediction.views.items():
                frame = usable_frames[view]
                for head, output in value.probabilities.items():
                    if head in self.categorical_heads:
                        if not self._publish_unless_shutdown(
                            self.publishers[view][head],
                            self._image_message(output, frame, categorical=True)
                        ):
                            return
                        if not self._publish_unless_shutdown(
                            self.confidence_publishers[view][head],
                            self._image_message(
                                value.categorical_confidences[head], frame
                            ),
                        ):
                            return
                    else:
                        if not self._publish_unless_shutdown(
                            self.publishers[view][head],
                            self._image_message(output, frame)
                        ):
                            return
                if publish_model_grid and view in self.model_grid_publishers:
                    class_publisher = self.model_grid_publishers[view]
                    confidence_publisher = self.model_grid_confidence_publishers[view]
                    if (
                        class_publisher.get_num_connections() > 0
                        or confidence_publisher.get_num_connections() > 0
                    ):
                        if not self._publish_unless_shutdown(
                            class_publisher,
                            self._image_message(
                                value.model_grid_probabilities["road_marking"],
                                frame,
                                categorical=True,
                            ),
                        ):
                            return
                        if not self._publish_unless_shutdown(
                            confidence_publisher,
                            self._image_message(
                                value.model_grid_categorical_confidences[
                                    "road_marking"
                                ],
                                frame,
                            ),
                        ):
                            return
            publish_ms = (time.perf_counter() - started) * 1000.0
            completed_ros_ns = int(rospy.Time.now().to_nsec())
            completed_monotonic = time.monotonic()
            with self._stats_lock:
                self.output_batches += 1
                self.total_output_batches += 1
                self.output_frames += len(prediction.views)
                for view in prediction.views:
                    self.output_by_view[view] += 1
                self._latest_result_stamp_ns = completed_ros_ns
                for view, frame in usable_frames.items():
                    stamp_ns = int(frame.stamp.to_nsec())
                    self._latest_input_stamp_ns[view] = stamp_ns
                    self._latest_receive_stamp_ns[view] = frame.received_ros_ns
                    self._queue_age_samples.append(
                        max(0.0, (completed_monotonic - frame.received_monotonic) * 1000.0)
                    )
                    end_to_end_ms = (completed_ros_ns - stamp_ns) / 1000000.0
                    if 0.0 <= end_to_end_ms <= 60000.0:
                        self._end_to_end_samples.append(end_to_end_ms)
                for name, value in (
                    ("decode_ms", decode_ms),
                    ("preprocess_ms", prediction.preprocess_ms),
                    ("inference_ms", prediction.inference_ms),
                    ("postprocess_ms", prediction.postprocess_ms),
                    ("publish_ms", publish_ms),
                ):
                    self.timing_totals[name] += value
                self._maybe_log()

    def _should_publish_model_grid(self, predictions, usable_frames):
        if not self.model_grid_publishers:
            return False
        if set(predictions) != set(self.views):
            self.model_grid_incomplete_batches += 1
            return False
        if not any(
            self.model_grid_publishers[view].get_num_connections() > 0
            or self.model_grid_confidence_publishers[view].get_num_connections() > 0
            for view in self.views
        ):
            return False
        stamp_ns = max(int(usable_frames[view].stamp.to_nsec()) for view in self.views)
        if stamp_ns <= 0:
            self.model_grid_incomplete_batches += 1
            return False
        if (
            self.last_model_grid_stamp_ns
            and stamp_ns + self.model_grid_period_ns <= self.last_model_grid_stamp_ns
        ):
            # Sequential bags and simulator reloads may start a new source-time
            # epoch. Preserve the header but reset the rate deadline.
            self.next_model_grid_stamp_ns = 0
            self.model_grid_timestamp_resets += 1
        elif self.last_model_grid_stamp_ns and stamp_ns <= self.last_model_grid_stamp_ns:
            self.model_grid_throttled_batches += 1
            return False
        self.last_model_grid_stamp_ns = stamp_ns
        if self.next_model_grid_stamp_ns and stamp_ns < self.next_model_grid_stamp_ns:
            self.model_grid_throttled_batches += 1
            return False
        self.next_model_grid_stamp_ns = stamp_ns + self.model_grid_period_ns
        self.model_grid_output_batches += 1
        return True

    def _model_grid_has_subscribers(self):
        return bool(self.model_grid_publishers) and any(
            self.model_grid_publishers[view].get_num_connections() > 0
            or self.model_grid_confidence_publishers[view].get_num_connections() > 0
            for view in self.views
        )

    def _maybe_log(self) -> None:
        now = time.monotonic()
        elapsed = now - self._last_log_wall
        if elapsed < self.log_interval:
            return
        frame_delta = self.output_frames - self._last_output_frames
        divisor = max(1, self.output_batches)
        averages = {
            name: round(value / divisor, 2)
            for name, value in self.timing_totals.items()
        }
        decode_delta = self.decode_errors - self._last_decode_errors
        inference_delta = self.inference_errors - self._last_inference_errors
        fps = frame_delta / elapsed
        rospy.loginfo(
            "semantic fps=%.2f output=%d batches=%d replaced=%s decode_errors=%d inference_errors=%d avg_ms=%s",
            fps,
            self.output_frames,
            self.output_batches,
            self.buffer.replaced,
            self.decode_errors,
            self.inference_errors,
            averages,
        )
        self._publish_diagnostics(
            fps, averages, decode_delta, inference_delta
        )
        self._last_log_wall = now
        self._last_output_frames = self.output_frames
        self._last_decode_errors = self.decode_errors
        self._last_inference_errors = self.inference_errors
        self.output_batches = 0
        for name in self.timing_totals:
            self.timing_totals[name] = 0.0
        self._queue_age_samples = []
        self._end_to_end_samples = []

    @staticmethod
    def _percentile(samples, value):
        if not samples:
            return None
        return float(np.percentile(np.asarray(samples, dtype=np.float64), value))

    @staticmethod
    def _diagnostic_value(key, value):
        if isinstance(value, (dict, list, tuple)):
            rendered = json.dumps(value, sort_keys=True)
        elif value is None:
            rendered = "unavailable"
        else:
            rendered = str(value)
        return KeyValue(key=str(key), value=rendered)

    def _publish_diagnostics(self, fps, averages, decode_delta, inference_delta):
        status = DiagnosticStatus()
        status.name = "camera_semantic_perception/inference"
        status.hardware_id = str(self.backend_info.get("device", "unknown"))
        if inference_delta:
            status.level = DiagnosticStatus.ERROR
            status.message = "inference errors"
        elif decode_delta:
            status.level = DiagnosticStatus.WARN
            status.message = "decode errors"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "running"

        values = {
            "backend": self.backend_info.get("name"),
            "model_config": self.backend_info.get("model_config"),
            "checkpoint_path": self.backend_info.get("checkpoint_path"),
            "checkpoint_sha256": self.backend_info.get("checkpoint_sha256"),
            "weights": self.backend_info.get("weights"),
            "views": self.views,
            "input_hw": self.backend_info.get("input_hw"),
            "received_by_view": self.buffer.received,
            "replaced_by_view": self.buffer.replaced,
            "output_frames": self.output_frames,
            "output_by_view": self.output_by_view,
            "output_batches": self.total_output_batches,
            "model_grid_output_batches": self.model_grid_output_batches,
            "model_grid_throttled_batches": self.model_grid_throttled_batches,
            "model_grid_incomplete_batches": self.model_grid_incomplete_batches,
            "model_grid_timestamp_resets": self.model_grid_timestamp_resets,
            "decode_errors": self.decode_errors,
            "inference_errors": self.inference_errors,
            "processing_fps": round(fps, 3),
            "average_stage_ms": averages,
            "queue_age_p50_ms": self._percentile(self._queue_age_samples, 50),
            "queue_age_p95_ms": self._percentile(self._queue_age_samples, 95),
            "end_to_end_p50_ms": self._percentile(self._end_to_end_samples, 50),
            "end_to_end_p95_ms": self._percentile(self._end_to_end_samples, 95),
            "latest_input_stamp_ns": self._latest_input_stamp_ns,
            "latest_receive_stamp_ns": self._latest_receive_stamp_ns,
            "latest_result_stamp_ns": self._latest_result_stamp_ns,
            "source_stamp_zero_by_view": self._source_stamp_zero_by_view,
        }
        status.values = [
            self._diagnostic_value(key, value) for key, value in values.items()
        ]
        message = DiagnosticArray()
        message.header.stamp = rospy.Time.now()
        message.status = [status]
        self.diagnostics_publisher.publish(message)

    def _shutdown(self) -> None:
        self.buffer.close()


def main() -> None:
    rospy.init_node("camera_semantic_inference")
    SemanticInferenceNode()
    rospy.spin()


if __name__ == "__main__":
    main()
