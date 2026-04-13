#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
import threading
import time
import traceback
from collections import deque
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse


DEFAULT_ROOM_WIDTH_M = 6.391
DEFAULT_ROOM_LENGTH_M = 12.468
DEFAULT_MAX_TRAIL_POINTS = 1500
DEFAULT_MAX_DIAGNOSTIC_MESSAGES = 24
DEFAULT_DIRECT_FREQ_HZ = 120.0
ASSET_DIR = Path(__file__).resolve().parent / "web_live"
REPO_ROOT = Path(__file__).resolve().parents[1]
ROS2_DIR = REPO_ROOT / "Ros_2"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Low-latency browser visualizer for PhaseSpace data. "
            "It can subscribe to ROS 2 /phasespace or connect directly to the PhaseSpace server."
        )
    )
    parser.add_argument(
        "--source",
        choices=("auto", "ros2", "direct"),
        default="auto",
        help="Data source. Auto chooses direct when --device is provided, otherwise ROS 2.",
    )
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="HTTP bind address for the local visualizer server.",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8765,
        help="HTTP port for the local visualizer server.",
    )
    parser.add_argument(
        "--topic",
        default="/phasespace",
        help="ROS 2 pose topic used when --source=ros2.",
    )
    parser.add_argument(
        "--velocity-topic",
        default="/phasespace_vel",
        help="ROS 2 velocity topic used when --source=ros2.",
    )
    parser.add_argument(
        "--rosout-topic",
        default="/rosout",
        help="ROS 2 rosout topic used when --source=ros2.",
    )
    parser.add_argument(
        "--parameter-events-topic",
        default="/parameter_events",
        help="ROS 2 parameter events topic used when --source=ros2.",
    )
    parser.add_argument(
        "--device",
        default=None,
        help="PhaseSpace host used when --source=direct.",
    )
    parser.add_argument(
        "--freq",
        type=float,
        default=DEFAULT_DIRECT_FREQ_HZ,
        help="Requested PhaseSpace streaming frequency in direct mode.",
    )
    parser.add_argument(
        "--timeout",
        type=int,
        default=2000,
        help="Direct-mode OWL nextEvent timeout in microseconds.",
    )
    parser.add_argument(
        "--stream",
        choices=("tcp", "udp"),
        default="udp",
        help="Direct-mode PhaseSpace transport.",
    )
    parser.add_argument(
        "--jsonfiles",
        nargs="*",
        default=None,
        help="Tracker JSON files for direct mode when not using server-side trackers.",
    )
    parser.add_argument(
        "--use-server-trackers",
        action="store_true",
        help="Use tracker definitions already active on the PhaseSpace server in direct mode.",
    )
    parser.add_argument(
        "--position-scale",
        type=float,
        default=0.001,
        help="Scale applied to incoming positions. 0.001 converts millimeters to meters.",
    )
    parser.add_argument(
        "--max-trail-points",
        type=int,
        default=DEFAULT_MAX_TRAIL_POINTS,
        help="Maximum number of points retained per tracker trail.",
    )
    parser.add_argument(
        "--room-anchor",
        choices=("trajectory_center", "center", "corner"),
        default="trajectory_center",
        help="How to place the room overlay in the browser view.",
    )
    parser.add_argument(
        "--room-origin-x-m",
        type=float,
        default=0.0,
        help="Room x origin in meters when room-anchor is center or corner.",
    )
    parser.add_argument(
        "--room-origin-z-m",
        type=float,
        default=0.0,
        help="Room z origin in meters when room-anchor is center or corner.",
    )
    parser.add_argument(
        "--room-long-axis",
        choices=("auto", "x", "z"),
        default="auto",
        help="Assign the room's long side to world x or z. Auto follows the larger observed span.",
    )
    parser.add_argument(
        "--room-width-m",
        type=float,
        default=DEFAULT_ROOM_WIDTH_M,
        help="Short side of the room footprint in meters.",
    )
    parser.add_argument(
        "--room-length-m",
        type=float,
        default=DEFAULT_ROOM_LENGTH_M,
        help="Long side of the room footprint in meters.",
    )
    parser.add_argument(
        "--hide-room",
        action="store_true",
        help="Render trajectories without the room overlay.",
    )
    return parser.parse_args()


def resolve_source(args: argparse.Namespace) -> str:
    if args.source != "auto":
        return args.source
    return "direct" if args.device else "ros2"


@dataclass(frozen=True)
class LiveSample:
    tracker_id: str
    stamp_ns: int
    x_m: float
    y_m: float
    z_m: float
    qw: float
    qx: float
    qy: float
    qz: float


@dataclass(frozen=True)
class LiveVelocity:
    tracker_id: str
    stamp_ns: int
    vx_mps: float
    vy_mps: float
    vz_mps: float
    speed_mps: float


@dataclass
class TrackerSeries:
    samples: deque[LiveSample]
    sample_count: int = 0
    last_update_ns: int = 0
    latest_velocity: LiveVelocity | None = None
    velocity_count: int = 0
    last_velocity_update_ns: int = 0


class SharedState:
    def __init__(self, args: argparse.Namespace, source: str) -> None:
        self.args = args
        self.source = source
        self._condition = threading.Condition()
        self._sequence = 0
        self._closed = False
        self._trackers: dict[str, TrackerSeries] = {}
        self._history: deque[dict[str, Any]] = deque(maxlen=4096)
        self._pose_frame_times: deque[float] = deque(maxlen=600)
        self._velocity_frame_times: deque[float] = deque(maxlen=600)
        self._total_pose_frames = 0
        self._total_velocity_frames = 0
        self._total_samples = 0
        self._status = "starting"
        self._last_error = ""
        self._started_ns = time.time_ns()
        self._last_pose_received_ns = 0
        self._last_velocity_received_ns = 0
        self._recent_logs: deque[dict[str, Any]] = deque(
            maxlen=DEFAULT_MAX_DIAGNOSTIC_MESSAGES
        )
        self._recent_parameter_events: deque[dict[str, Any]] = deque(
            maxlen=DEFAULT_MAX_DIAGNOSTIC_MESSAGES
        )
        self._rosout_count = 0
        self._parameter_event_count = 0

    def _ensure_tracker(self, tracker_id: str) -> TrackerSeries:
        tracker = self._trackers.get(tracker_id)
        if tracker is None:
            tracker = TrackerSeries(samples=deque(maxlen=self.args.max_trail_points))
            self._trackers[tracker_id] = tracker
        return tracker

    def _rate_hz(self, samples: deque[float]) -> float:
        if len(samples) < 2:
            return 0.0
        dt = samples[-1] - samples[0]
        return (len(samples) - 1) / dt if dt > 0.0 else 0.0

    def set_status(self, status: str, error: str | None = None) -> None:
        with self._condition:
            self._status = status
            if error is not None:
                self._last_error = error
            self._condition.notify_all()

    def close(self) -> None:
        with self._condition:
            self._closed = True
            self._condition.notify_all()

    def publish_samples(self, samples: list[LiveSample]) -> None:
        if not samples:
            return

        now_ns = time.time_ns()
        now_perf = time.perf_counter()

        with self._condition:
            event_samples: list[dict[str, Any]] = []
            for sample in samples:
                tracker = self._ensure_tracker(sample.tracker_id)
                previous = tracker.samples[-1] if tracker.samples else None
                tracker.samples.append(sample)
                tracker.sample_count += 1
                tracker.last_update_ns = now_ns
                self._total_samples += 1
                if (
                    previous is not None
                    and sample.stamp_ns > previous.stamp_ns
                    and (
                        tracker.latest_velocity is None
                        or sample.stamp_ns - tracker.last_velocity_update_ns > 250_000_000
                    )
                ):
                    dt = (sample.stamp_ns - previous.stamp_ns) / 1e9
                    vx_mps = (sample.x_m - previous.x_m) / dt
                    vy_mps = (sample.y_m - previous.y_m) / dt
                    vz_mps = (sample.z_m - previous.z_m) / dt
                    tracker.latest_velocity = LiveVelocity(
                        tracker_id=sample.tracker_id,
                        stamp_ns=sample.stamp_ns,
                        vx_mps=vx_mps,
                        vy_mps=vy_mps,
                        vz_mps=vz_mps,
                        speed_mps=(vx_mps**2 + vy_mps**2 + vz_mps**2) ** 0.5,
                    )
                    tracker.last_velocity_update_ns = sample.stamp_ns

                payload = self._sample_payload(sample)
                if tracker.latest_velocity is not None:
                    payload["velocity"] = self._velocity_payload(
                        tracker.latest_velocity
                    )
                event_samples.append(payload)

            self._total_pose_frames += 1
            self._pose_frame_times.append(now_perf)
            self._last_pose_received_ns = now_ns
            self._sequence += 1
            event = {
                "type": "delta",
                "sequence": self._sequence,
                "server_time_ns": now_ns,
                "stats": self._stats_payload(now_ns),
                "samples": event_samples,
            }
            self._history.append(event)
            self._condition.notify_all()

    def publish_velocity(self, velocity: LiveVelocity) -> None:
        now_ns = time.time_ns()
        now_perf = time.perf_counter()
        with self._condition:
            tracker = self._ensure_tracker(velocity.tracker_id)
            tracker.latest_velocity = velocity
            tracker.velocity_count += 1
            tracker.last_velocity_update_ns = velocity.stamp_ns or now_ns
            self._total_velocity_frames += 1
            self._velocity_frame_times.append(now_perf)
            self._last_velocity_received_ns = now_ns
            self._sequence += 1
            event = {
                "type": "velocity",
                "sequence": self._sequence,
                "server_time_ns": now_ns,
                "stats": self._stats_payload(now_ns),
                "velocity": self._velocity_payload(velocity),
            }
            self._history.append(event)
            self._condition.notify_all()

    def publish_log(
        self, *, stamp_ns: int, level: int, name: str, message: str
    ) -> None:
        now_ns = time.time_ns()
        with self._condition:
            entry = {
                "stamp_ns": stamp_ns or now_ns,
                "level": int(level),
                "name": name or "unknown",
                "message": message,
            }
            self._recent_logs.appendleft(entry)
            self._rosout_count += 1
            self._sequence += 1
            event = {
                "type": "log",
                "sequence": self._sequence,
                "server_time_ns": now_ns,
                "stats": self._stats_payload(now_ns),
                "log": entry,
            }
            self._history.append(event)
            self._condition.notify_all()

    def publish_parameter_event(
        self,
        *,
        stamp_ns: int,
        node: str,
        new_parameters: list[str],
        changed_parameters: list[str],
        deleted_parameters: list[str],
    ) -> None:
        now_ns = time.time_ns()
        with self._condition:
            entry = {
                "stamp_ns": stamp_ns or now_ns,
                "node": node or "unknown",
                "new_parameters": new_parameters,
                "changed_parameters": changed_parameters,
                "deleted_parameters": deleted_parameters,
            }
            self._recent_parameter_events.appendleft(entry)
            self._parameter_event_count += 1
            self._sequence += 1
            event = {
                "type": "parameter_event",
                "sequence": self._sequence,
                "server_time_ns": now_ns,
                "stats": self._stats_payload(now_ns),
                "parameter_event": entry,
            }
            self._history.append(event)
            self._condition.notify_all()

    def _sample_payload(self, sample: LiveSample) -> dict[str, Any]:
        return {
            "id": sample.tracker_id,
            "stamp_ns": sample.stamp_ns,
            "x_m": round(sample.x_m, 6),
            "y_m": round(sample.y_m, 6),
            "z_m": round(sample.z_m, 6),
            "qw": round(sample.qw, 6),
            "qx": round(sample.qx, 6),
            "qy": round(sample.qy, 6),
            "qz": round(sample.qz, 6),
        }

    def _velocity_payload(self, velocity: LiveVelocity) -> dict[str, Any]:
        return {
            "id": velocity.tracker_id,
            "stamp_ns": velocity.stamp_ns,
            "vx_mps": round(velocity.vx_mps, 6),
            "vy_mps": round(velocity.vy_mps, 6),
            "vz_mps": round(velocity.vz_mps, 6),
            "speed_mps": round(velocity.speed_mps, 6),
        }

    def _stats_payload(self, now_ns: int | None = None) -> dict[str, Any]:
        if now_ns is None:
            now_ns = time.time_ns()

        pose_hz = self._rate_hz(self._pose_frame_times)
        velocity_hz = self._rate_hz(self._velocity_frame_times)
        last_pose_age_ms = (
            round((now_ns - self._last_pose_received_ns) / 1e6, 2)
            if self._last_pose_received_ns
            else None
        )
        last_velocity_age_ms = (
            round((now_ns - self._last_velocity_received_ns) / 1e6, 2)
            if self._last_velocity_received_ns
            else None
        )

        return {
            "ingest_hz": round(pose_hz, 2),
            "pose_hz": round(pose_hz, 2),
            "velocity_hz": round(velocity_hz, 2),
            "tracker_count": len(self._trackers),
            "total_frames": self._total_pose_frames,
            "total_pose_frames": self._total_pose_frames,
            "total_velocity_frames": self._total_velocity_frames,
            "total_samples": self._total_samples,
            "rosout_count": self._rosout_count,
            "parameter_event_count": self._parameter_event_count,
            "uptime_s": round((now_ns - self._started_ns) / 1e9, 2),
            "last_frame_age_ms": last_pose_age_ms,
            "last_pose_age_ms": last_pose_age_ms,
            "last_velocity_age_ms": last_velocity_age_ms,
            "status": self._status,
            "last_error": self._last_error,
        }

    def _config_payload(self) -> dict[str, Any]:
        source_detail = {
            "mode": self.source,
            "device": self.args.device if self.source == "direct" else None,
            "stream": self.args.stream if self.source == "direct" else None,
            "freq_hz": self.args.freq if self.source == "direct" else None,
        }
        return {
            "source": source_detail,
            "topics": {
                "pose": self.args.topic if self.source == "ros2" else None,
                "velocity": self.args.velocity_topic if self.source == "ros2" else None,
                "rosout": self.args.rosout_topic if self.source == "ros2" else None,
                "parameter_events": (
                    self.args.parameter_events_topic if self.source == "ros2" else None
                ),
            },
            "maxTrailPoints": self.args.max_trail_points,
            "positionScale": self.args.position_scale,
            "room": {
                "hide": self.args.hide_room,
                "anchor": self.args.room_anchor,
                "originX": self.args.room_origin_x_m,
                "originZ": self.args.room_origin_z_m,
                "widthM": self.args.room_width_m,
                "lengthM": self.args.room_length_m,
                "longAxis": self.args.room_long_axis,
            },
        }

    def snapshot(self) -> dict[str, Any]:
        with self._condition:
            trackers = []
            for tracker_id in sorted(self._trackers):
                tracker = self._trackers[tracker_id]
                latest = tracker.samples[-1] if tracker.samples else None
                trackers.append(
                    {
                        "id": tracker_id,
                        "sample_count": tracker.sample_count,
                        "velocity_count": tracker.velocity_count,
                        "latest": self._sample_payload(latest) if latest else None,
                        "latest_velocity": (
                            self._velocity_payload(tracker.latest_velocity)
                            if tracker.latest_velocity is not None
                            else None
                        ),
                        "trail": [
                            self._sample_payload(sample) for sample in tracker.samples
                        ],
                    }
                )

            return {
                "type": "snapshot",
                "sequence": self._sequence,
                "server_time_ns": time.time_ns(),
                "config": self._config_payload(),
                "stats": self._stats_payload(),
                "diagnostics": {
                    "logs": list(self._recent_logs),
                    "parameter_events": list(self._recent_parameter_events),
                },
                "trackers": trackers,
            }

    def wait_for_events(
        self, after_sequence: int, timeout_s: float = 5.0
    ) -> list[dict[str, Any]] | None:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while not self._closed and self._sequence <= after_sequence:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return []
                self._condition.wait(timeout=remaining)

            if self._closed:
                return None

            if not self._history:
                return []

            earliest_sequence = self._history[0]["sequence"]
            if after_sequence and after_sequence < earliest_sequence - 1:
                return [self.snapshot()]

            return [
                event for event in self._history if event["sequence"] > after_sequence
            ]


class StaticAndEventHandler(BaseHTTPRequestHandler):
    state: SharedState
    asset_dir: Path

    def do_GET(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        if path in {"/", "/index.html"}:
            return self._serve_file("index.html", "text/html; charset=utf-8")
        if path == "/app.js":
            return self._serve_file("app.js", "application/javascript; charset=utf-8")
        if path == "/styles.css":
            return self._serve_file("styles.css", "text/css; charset=utf-8")
        if path == "/snapshot":
            return self._serve_json(self.state.snapshot())
        if path == "/health":
            return self._serve_json(
                {
                    "ok": True,
                    "stats": self.state.snapshot()["stats"],
                }
            )
        if path == "/events":
            return self._serve_events()
        self.send_error(404, "Not Found")

    def _serve_file(self, name: str, content_type: str) -> None:
        asset_path = self.asset_dir / name
        if not asset_path.is_file():
            self.send_error(404, "Missing asset")
            return
        data = asset_path.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def _serve_json(self, payload: dict[str, Any]) -> None:
        data = json.dumps(payload).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def _write_sse(self, event_name: str, payload: dict[str, Any]) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.wfile.write(b"event: " + event_name.encode("utf-8") + b"\n")
        self.wfile.write(b"data: " + body + b"\n\n")
        self.wfile.flush()

    def _serve_events(self) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()

        last_sequence = 0
        try:
            snapshot = self.state.snapshot()
            self._write_sse("snapshot", snapshot)
            last_sequence = snapshot["sequence"]
            while True:
                events = self.state.wait_for_events(last_sequence, timeout_s=5.0)
                if events is None:
                    return
                if not events:
                    self.wfile.write(b": keepalive\n\n")
                    self.wfile.flush()
                    continue
                for event in events:
                    self._write_sse(event["type"], event)
                    last_sequence = event["sequence"]
        except (BrokenPipeError, ConnectionResetError):
            return

    def log_message(self, format: str, *args: Any) -> None:
        del format, args


class DirectPhaseSpaceRunner:
    def __init__(self, args: argparse.Namespace, state: SharedState) -> None:
        self.args = args
        self.state = state
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._stop_event = threading.Event()

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        self._thread.join(timeout=2.0)

    def _run(self) -> None:
        if not self.args.device:
            self.state.set_status(
                "error", "Direct mode requires --device <phasespace-host>."
            )
            return

        sys.path.insert(0, str(ROS2_DIR))
        try:
            from phasespace_node_ros2 import Context, Type, load_tracker_configs
        except Exception as exc:
            self.state.set_status(
                "error", f"Failed to import PhaseSpace client code: {exc}"
            )
            return

        owl = None
        self.state.set_status("connecting")
        try:
            owl = Context()
            owl.open(self.args.device, "timeout=5000000")
            owl.initialize("timeout=5000000 slave=0 event.cameras=0 event.markers=0 event.rigids=1")
            if self.args.freq:
                owl.frequency(self.args.freq)
            owl.streaming(2 if self.args.stream == "udp" else 1)

            if self.args.use_server_trackers:
                tracker_configs = []
            else:
                if not self.args.jsonfiles:
                    raise ValueError(
                        "Direct mode requires --jsonfiles ... or --use-server-trackers."
                    )
                tracker_configs = load_tracker_configs(self.args.jsonfiles)

            for tracker_config in tracker_configs:
                owl.createTracker(
                    tracker_config["tracker_id"],
                    "rigid",
                    tracker_config["name"],
                )
                for marker in tracker_config["markers"]:
                    owl.assignMarker(
                        tracker_config["tracker_id"],
                        marker["id"],
                        marker["name"],
                        marker["options"],
                    )

            self.state.set_status("streaming")
            while not self._stop_event.is_set() and owl.isOpen() and owl.property(
                "initialized"
            ):
                event = owl.nextEvent(self.args.timeout)
                if event is None:
                    continue
                if event.type_id == Type.ERROR or getattr(event, "name", None) == "done":
                    self.state.set_status("error", "PhaseSpace stream ended.")
                    break
                if event.type_id != Type.FRAME or "rigids" not in event:
                    continue

                frame_samples: list[LiveSample] = []
                for rigid in event.rigids:
                    if rigid.cond <= 0:
                        continue
                    pose = rigid.pose
                    frame_samples.append(
                        LiveSample(
                            tracker_id=str(rigid.id),
                            stamp_ns=time.time_ns(),
                            x_m=pose[0] * self.args.position_scale,
                            y_m=pose[1] * self.args.position_scale,
                            z_m=pose[2] * self.args.position_scale,
                            qw=pose[3],
                            qx=pose[4],
                            qy=pose[5],
                            qz=pose[6],
                        )
                    )

                self.state.publish_samples(frame_samples)
        except Exception as exc:
            self.state.set_status("error", f"Direct PhaseSpace ingest failed: {exc}")
            traceback.print_exc()
        finally:
            if owl is not None:
                try:
                    owl.done()
                except Exception:
                    pass
                try:
                    owl.close()
                except Exception:
                    pass


class Ros2PhaseSpaceRunner:
    def __init__(self, args: argparse.Namespace, state: SharedState) -> None:
        self.args = args
        self.state = state
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._stop_event = threading.Event()
        self._executor = None
        self._node = None
        self._rclpy = None

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._executor is not None:
            self._executor.shutdown()
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
        if self._rclpy is not None:
            try:
                self._rclpy.shutdown()
            except Exception:
                pass
        self._thread.join(timeout=2.0)

    def _run(self) -> None:
        self.state.set_status("connecting")
        try:
            import rclpy
            from geometry_msgs.msg import TransformStamped, TwistStamped
            from rcl_interfaces.msg import Log, ParameterEvent
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import (
                qos_profile_parameter_events,
                qos_profile_sensor_data,
                qos_profile_system_default,
            )
        except Exception as exc:
            self.state.set_status("error", f"ROS 2 imports failed: {exc}")
            return

        self._rclpy = rclpy
        rclpy.init()

        args = self.args
        state = self.state

        class Ros2IngestNode(Node):
            def __init__(self) -> None:
                super().__init__("phasespace_live_visualizer_web")
                self.pose_subscription = self.create_subscription(
                    TransformStamped,
                    args.topic,
                    self._handle_pose,
                    qos_profile_sensor_data,
                )
                self.velocity_subscription = self.create_subscription(
                    TwistStamped,
                    args.velocity_topic,
                    self._handle_velocity,
                    qos_profile_sensor_data,
                )
                self.rosout_subscription = self.create_subscription(
                    Log,
                    args.rosout_topic,
                    self._handle_log,
                    qos_profile_system_default,
                )
                self.parameter_subscription = self.create_subscription(
                    ParameterEvent,
                    args.parameter_events_topic,
                    self._handle_parameter_event,
                    qos_profile_parameter_events,
                )

            def _handle_pose(self, message: TransformStamped) -> None:
                stamp_ns = (
                    message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
                )
                state.publish_samples(
                    [
                        LiveSample(
                            tracker_id=message.child_frame_id or "unknown",
                            stamp_ns=stamp_ns or time.time_ns(),
                            x_m=message.transform.translation.x * args.position_scale,
                            y_m=message.transform.translation.y * args.position_scale,
                            z_m=message.transform.translation.z * args.position_scale,
                            qw=message.transform.rotation.w,
                            qx=message.transform.rotation.x,
                            qy=message.transform.rotation.y,
                            qz=message.transform.rotation.z,
                        )
                    ]
                )

            def _handle_velocity(self, message: TwistStamped) -> None:
                stamp_ns = (
                    message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
                )
                vx_mps = float(message.twist.linear.x)
                vy_mps = float(message.twist.linear.y)
                vz_mps = float(message.twist.linear.z)
                state.publish_velocity(
                    LiveVelocity(
                        tracker_id=message.header.frame_id or "unknown",
                        stamp_ns=stamp_ns or time.time_ns(),
                        vx_mps=vx_mps,
                        vy_mps=vy_mps,
                        vz_mps=vz_mps,
                        speed_mps=(vx_mps**2 + vy_mps**2 + vz_mps**2) ** 0.5,
                    )
                )

            def _handle_log(self, message: Log) -> None:
                stamp_ns = (
                    message.stamp.sec * 10**9 + message.stamp.nanosec
                    if hasattr(message, "stamp")
                    else time.time_ns()
                )
                state.publish_log(
                    stamp_ns=stamp_ns,
                    level=int(message.level),
                    name=message.name,
                    message=message.msg,
                )

            def _handle_parameter_event(self, message: ParameterEvent) -> None:
                stamp_ns = (
                    message.stamp.sec * 10**9 + message.stamp.nanosec
                )
                state.publish_parameter_event(
                    stamp_ns=stamp_ns,
                    node=message.node,
                    new_parameters=[parameter.name for parameter in message.new_parameters],
                    changed_parameters=[
                        parameter.name for parameter in message.changed_parameters
                    ],
                    deleted_parameters=[
                        parameter.name for parameter in message.deleted_parameters
                    ],
                )

        try:
            self._node = Ros2IngestNode()
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)
            self.state.set_status("streaming")
            self._executor.spin()
        except Exception as exc:
            self.state.set_status("error", f"ROS 2 ingest failed: {exc}")
            traceback.print_exc()
        finally:
            if self._executor is not None:
                try:
                    self._executor.shutdown()
                except Exception:
                    pass
            if self._node is not None:
                try:
                    self._node.destroy_node()
                except Exception:
                    pass
            try:
                rclpy.shutdown()
            except Exception:
                pass


def make_handler(
    state: SharedState, asset_dir: Path
) -> type[StaticAndEventHandler]:
    class BoundHandler(StaticAndEventHandler):
        pass

    BoundHandler.state = state
    BoundHandler.asset_dir = asset_dir
    return BoundHandler


def main() -> int:
    args = parse_args()
    source = resolve_source(args)
    state = SharedState(args=args, source=source)

    if source == "direct" and not args.device:
        raise SystemExit("--source=direct requires --device.")

    if not ASSET_DIR.is_dir():
        raise SystemExit(f"Missing asset directory: {ASSET_DIR}")

    server = ThreadingHTTPServer((args.host, args.port), make_handler(state, ASSET_DIR))
    server.daemon_threads = True
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()

    runner = (
        DirectPhaseSpaceRunner(args, state)
        if source == "direct"
        else Ros2PhaseSpaceRunner(args, state)
    )
    runner.start()

    bound_host, bound_port = server.server_address[:2]
    print(
        f"[PhaseSpace] Fast live visualizer ready at http://{bound_host}:{bound_port}/ "
        f"(source={source})"
    )
    if source == "direct":
        print(
            f"[PhaseSpace] Direct mode uses {args.stream.upper()} at requested {args.freq:.1f} Hz."
        )
    else:
        print(
            "[PhaseSpace] ROS 2 mode subscribed to "
            f"{args.topic}, {args.velocity_topic}, {args.parameter_events_topic}, "
            f"and {args.rosout_topic}."
        )

    try:
        while server_thread.is_alive():
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        state.set_status("stopping")
        state.close()
        runner.stop()
        server.shutdown()
        server.server_close()
        server_thread.join(timeout=2.0)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
