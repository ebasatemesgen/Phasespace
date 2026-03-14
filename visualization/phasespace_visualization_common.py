#!/usr/bin/env python3
from __future__ import annotations

import os
import struct
import tempfile
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, Iterator, Mapping, Sequence

os.environ.setdefault(
    "MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "phasespace_matplotlib")
)

import numpy as np
from geometry_msgs.msg import TransformStamped
from rclpy.serialization import deserialize_message

MCAP_MAGIC = b"\x89MCAP0\r\n"

OP_CHANNEL = 0x04
OP_CHUNK = 0x06
OP_MESSAGE = 0x05

OPEN_LAB_WIDTH_M = 6.391
OPEN_LAB_LENGTH_M = 12.468
OPEN_LAB_SIDE_RECESS_M = 0.651
DEFAULT_MAX_HEIGHT_M = 3.0


@dataclass(frozen=True)
class PoseSample:
    stamp_ns: int
    tracker_id: str
    x_m: float
    y_m: float
    z_m: float
    qw: float
    qx: float
    qy: float
    qz: float


@dataclass(frozen=True)
class RoomGeometry:
    width_m: float = OPEN_LAB_WIDTH_M
    length_m: float = OPEN_LAB_LENGTH_M
    side_recess_m: float = OPEN_LAB_SIDE_RECESS_M
    max_height_m: float = DEFAULT_MAX_HEIGHT_M


def bag_name_from_path(path: str | os.PathLike[str]) -> str:
    bag_path = Path(path)
    return bag_path.name if bag_path.is_dir() else bag_path.stem


def resolve_mcap_path(path: str | os.PathLike[str]) -> Path:
    bag_path = Path(path)
    if bag_path.is_file() and bag_path.suffix == ".mcap":
        return bag_path
    if not bag_path.is_dir():
        raise FileNotFoundError(f"Bag path does not exist: {bag_path}")

    matches = sorted(bag_path.glob("*.mcap"))
    if not matches:
        raise FileNotFoundError(f"No .mcap file found in {bag_path}")
    if len(matches) > 1:
        raise RuntimeError(
            f"Expected one .mcap file in {bag_path}, found {len(matches)}"
        )
    return matches[0]


def _read_u16(buf: bytes, offset: int) -> tuple[int, int]:
    return struct.unpack_from("<H", buf, offset)[0], offset + 2


def _read_u32(buf: bytes, offset: int) -> tuple[int, int]:
    return struct.unpack_from("<I", buf, offset)[0], offset + 4


def _read_u64(buf: bytes, offset: int) -> tuple[int, int]:
    return struct.unpack_from("<Q", buf, offset)[0], offset + 8


def _read_str(buf: bytes, offset: int) -> tuple[str, int]:
    size, offset = _read_u32(buf, offset)
    return buf[offset : offset + size].decode("utf-8"), offset + size


def _iter_records(data: bytes) -> Iterator[tuple[int, bytes]]:
    offset = 0
    limit = len(data)
    while offset + 9 <= limit:
        opcode = data[offset]
        length = struct.unpack_from("<Q", data, offset + 1)[0]
        start = offset + 9
        end = start + length
        yield opcode, data[start:end]
        offset = end


def _parse_channel_record(data: bytes) -> dict[str, object]:
    offset = 0
    channel_id, offset = _read_u16(data, offset)
    schema_id, offset = _read_u16(data, offset)
    topic, offset = _read_str(data, offset)
    message_encoding, offset = _read_str(data, offset)
    metadata_length, offset = _read_u32(data, offset)
    metadata_end = offset + metadata_length
    metadata: dict[str, str] = {}
    while offset < metadata_end:
        key, offset = _read_str(data, offset)
        value, offset = _read_str(data, offset)
        metadata[key] = value
    return {
        "channel_id": channel_id,
        "schema_id": schema_id,
        "topic": topic,
        "message_encoding": message_encoding,
        "metadata": metadata,
    }


def _parse_message_record(data: bytes) -> tuple[int, int, int, int, bytes]:
    offset = 0
    channel_id, offset = _read_u16(data, offset)
    sequence, offset = _read_u32(data, offset)
    log_time_ns, offset = _read_u64(data, offset)
    publish_time_ns, offset = _read_u64(data, offset)
    return channel_id, sequence, log_time_ns, publish_time_ns, data[offset:]


def _iter_chunk_records(data: bytes) -> Iterator[tuple[int, bytes]]:
    offset = 0
    _, offset = _read_u64(data, offset)
    _, offset = _read_u64(data, offset)
    _, offset = _read_u64(data, offset)
    checksum, offset = _read_u32(data, offset)
    compression, offset = _read_str(data, offset)
    records_length, offset = _read_u64(data, offset)
    records = data[offset : offset + records_length]

    if compression:
        raise RuntimeError(
            "This visualizer only supports uncompressed MCAP chunks. "
            f"Found compression={compression!r}."
        )
    if checksum and (zlib.crc32(records) & 0xFFFFFFFF) != checksum:
        raise RuntimeError("MCAP chunk checksum mismatch.")

    yield from _iter_records(records)


def iter_mcap_messages(
    bag_path: str | os.PathLike[str],
    topics: Iterable[str] | None = None,
) -> Iterator[tuple[str, int, bytes]]:
    allowed_topics = set(topics) if topics else None
    channels: dict[int, dict[str, object]] = {}
    mcap_path = resolve_mcap_path(bag_path)
    raw = mcap_path.read_bytes()

    if not raw.startswith(MCAP_MAGIC) or not raw.endswith(MCAP_MAGIC):
        raise RuntimeError(f"{mcap_path} is not a valid MCAP file.")

    for opcode, record in _iter_records(raw[len(MCAP_MAGIC) : -len(MCAP_MAGIC)]):
        nested_records = _iter_chunk_records(record) if opcode == OP_CHUNK else [(opcode, record)]
        for nested_opcode, nested_record in nested_records:
            if nested_opcode == OP_CHANNEL:
                channel = _parse_channel_record(nested_record)
                channels[int(channel["channel_id"])] = channel
                continue
            if nested_opcode != OP_MESSAGE:
                continue

            channel_id, _, log_time_ns, _, payload = _parse_message_record(nested_record)
            topic = str(channels.get(channel_id, {}).get("topic", ""))
            if not topic:
                continue
            if allowed_topics and topic not in allowed_topics:
                continue
            yield topic, log_time_ns, payload


def load_transform_trajectories(
    bag_path: str | os.PathLike[str],
    topic: str = "/phasespace",
    position_scale: float = 0.001,
) -> dict[str, list[PoseSample]]:
    trajectories: dict[str, list[PoseSample]] = {}
    for _, log_time_ns, payload in iter_mcap_messages(bag_path, topics=[topic]):
        message = deserialize_message(payload, TransformStamped)
        tracker_id = message.child_frame_id or "unknown"
        sample = PoseSample(
            stamp_ns=log_time_ns,
            tracker_id=tracker_id,
            x_m=message.transform.translation.x * position_scale,
            y_m=message.transform.translation.y * position_scale,
            z_m=message.transform.translation.z * position_scale,
            qw=message.transform.rotation.w,
            qx=message.transform.rotation.x,
            qy=message.transform.rotation.y,
            qz=message.transform.rotation.z,
        )
        trajectories.setdefault(tracker_id, []).append(sample)

    for samples in trajectories.values():
        samples.sort(key=lambda sample: sample.stamp_ns)
    return dict(sorted(trajectories.items(), key=lambda item: item[0]))


def trajectory_arrays(samples: Sequence[PoseSample]) -> tuple[np.ndarray, np.ndarray]:
    if not samples:
        return np.array([], dtype=float), np.empty((0, 3), dtype=float)

    stamps_ns = np.array([sample.stamp_ns for sample in samples], dtype=np.int64)
    xyz = np.array(
        [[sample.x_m, sample.y_m, sample.z_m] for sample in samples], dtype=float
    )
    elapsed_s = (stamps_ns - stamps_ns[0]) / 1e9
    return elapsed_s, xyz


def compute_path_metrics(samples: Sequence[PoseSample]) -> dict[str, float]:
    elapsed_s, xyz = trajectory_arrays(samples)
    if len(samples) == 0:
        return {
            "sample_count": 0,
            "duration_s": 0.0,
            "distance_m": 0.0,
            "mean_speed_mps": 0.0,
            "max_speed_mps": 0.0,
            "min_height_m": 0.0,
            "max_height_m": 0.0,
            "x_span_m": 0.0,
            "z_span_m": 0.0,
        }

    deltas = np.diff(xyz, axis=0)
    step_lengths = np.linalg.norm(deltas, axis=1)
    step_times = np.diff(elapsed_s)
    speeds = np.divide(
        step_lengths,
        step_times,
        out=np.zeros_like(step_lengths),
        where=step_times > 0.0,
    )

    return {
        "sample_count": len(samples),
        "duration_s": float(elapsed_s[-1] if len(elapsed_s) else 0.0),
        "distance_m": float(step_lengths.sum()),
        "mean_speed_mps": float(speeds.mean() if len(speeds) else 0.0),
        "max_speed_mps": float(speeds.max() if len(speeds) else 0.0),
        "min_height_m": float(xyz[:, 1].min()),
        "max_height_m": float(xyz[:, 1].max()),
        "x_span_m": float(xyz[:, 0].max() - xyz[:, 0].min()),
        "z_span_m": float(xyz[:, 2].max() - xyz[:, 2].min()),
    }


def global_xyz_bounds(
    trajectories: Mapping[str, Sequence[PoseSample]]
) -> tuple[np.ndarray, np.ndarray]:
    all_points = [
        np.array([[sample.x_m, sample.y_m, sample.z_m] for sample in samples], dtype=float)
        for samples in trajectories.values()
        if samples
    ]
    if not all_points:
        raise ValueError("No trajectory samples available.")
    stacked = np.vstack(all_points)
    return stacked.min(axis=0), stacked.max(axis=0)


def resolve_room_axes(
    room: RoomGeometry,
    x_span_m: float,
    z_span_m: float,
    long_axis: str = "auto",
) -> tuple[float, float, str]:
    if long_axis not in {"auto", "x", "z"}:
        raise ValueError("long_axis must be one of: auto, x, z")

    resolved_axis = long_axis
    if resolved_axis == "auto":
        resolved_axis = "x" if x_span_m >= z_span_m else "z"

    if resolved_axis == "x":
        return room.length_m, room.width_m, resolved_axis
    return room.width_m, room.length_m, resolved_axis


def resolve_room_center(
    trajectories: Mapping[str, Sequence[PoseSample]],
    room_size_x_m: float,
    room_size_z_m: float,
    anchor: str = "trajectory_center",
    origin_x_m: float = 0.0,
    origin_z_m: float = 0.0,
) -> tuple[float, float]:
    if anchor not in {"trajectory_center", "center", "corner"}:
        raise ValueError("anchor must be one of: trajectory_center, center, corner")

    if anchor == "center":
        return origin_x_m, origin_z_m
    if anchor == "corner":
        return origin_x_m + room_size_x_m / 2.0, origin_z_m + room_size_z_m / 2.0

    try:
        min_xyz, max_xyz = global_xyz_bounds(trajectories)
    except ValueError:
        return origin_x_m, origin_z_m
    return (min_xyz[0] + max_xyz[0]) / 2.0, (min_xyz[2] + max_xyz[2]) / 2.0


def room_bounds_from_center(
    center_x_m: float, center_z_m: float, size_x_m: float, size_z_m: float
) -> tuple[float, float, float, float]:
    half_x = size_x_m / 2.0
    half_z = size_z_m / 2.0
    return (
        center_x_m - half_x,
        center_x_m + half_x,
        center_z_m - half_z,
        center_z_m + half_z,
    )


def draw_room_2d(
    ax,
    center_x_m: float,
    center_z_m: float,
    size_x_m: float,
    size_z_m: float,
    line_color: str = "#5c5c5c",
) -> tuple[float, float, float, float]:
    from matplotlib.patches import Rectangle

    xmin, xmax, zmin, zmax = room_bounds_from_center(
        center_x_m, center_z_m, size_x_m, size_z_m
    )
    ax.add_patch(
        Rectangle(
            (xmin, zmin),
            size_x_m,
            size_z_m,
            facecolor="#eee8dd",
            edgecolor=line_color,
            linewidth=1.6,
            linestyle="--",
            alpha=0.28,
            zorder=0,
        )
    )
    ax.plot([xmin, xmax], [center_z_m, center_z_m], color=line_color, alpha=0.18, linewidth=1.0)
    ax.plot([center_x_m, center_x_m], [zmin, zmax], color=line_color, alpha=0.18, linewidth=1.0)
    return xmin, xmax, zmin, zmax


def draw_room_3d(
    ax,
    center_x_m: float,
    center_z_m: float,
    size_x_m: float,
    size_z_m: float,
    max_height_m: float,
    line_color: str = "#6f6f6f",
) -> tuple[float, float, float, float]:
    xmin, xmax, zmin, zmax = room_bounds_from_center(
        center_x_m, center_z_m, size_x_m, size_z_m
    )
    floor = [
        (xmin, zmin, 0.0),
        (xmax, zmin, 0.0),
        (xmax, zmax, 0.0),
        (xmin, zmax, 0.0),
        (xmin, zmin, 0.0),
    ]
    ceiling = [(x, z, max_height_m) for x, z, _ in floor]

    for points in (floor, ceiling):
        xs = [point[0] for point in points]
        ys = [point[1] for point in points]
        zs = [point[2] for point in points]
        ax.plot(xs, ys, zs, color=line_color, linestyle="--", linewidth=1.1, alpha=0.7)

    for x, z in ((xmin, zmin), (xmax, zmin), (xmax, zmax), (xmin, zmax)):
        ax.plot(
            [x, x],
            [z, z],
            [0.0, max_height_m],
            color=line_color,
            linestyle="--",
            linewidth=1.0,
            alpha=0.5,
        )

    return xmin, xmax, zmin, zmax


def set_axes_equal_3d(ax) -> None:
    x_limits = np.array(ax.get_xlim3d(), dtype=float)
    y_limits = np.array(ax.get_ylim3d(), dtype=float)
    z_limits = np.array(ax.get_zlim3d(), dtype=float)

    spans = np.array(
        [
            abs(x_limits[1] - x_limits[0]),
            abs(y_limits[1] - y_limits[0]),
            abs(z_limits[1] - z_limits[0]),
        ]
    )
    spans = np.maximum(spans, 0.5)

    try:
        ax.set_box_aspect(tuple(spans))
        return
    except AttributeError:
        pass

    centers = np.array([x_limits.mean(), y_limits.mean(), z_limits.mean()])
    radius = max(spans.max() / 2.0, 0.5)
    ax.set_xlim3d(centers[0] - radius, centers[0] + radius)
    ax.set_ylim3d(centers[1] - radius, centers[1] + radius)
    ax.set_zlim3d(max(0.0, centers[2] - radius), centers[2] + radius)
