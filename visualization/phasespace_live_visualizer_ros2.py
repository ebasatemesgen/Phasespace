#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import site
import sys
import tempfile
import threading
from collections import deque
from pathlib import Path

os.environ.setdefault(
    "MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "phasespace_matplotlib")
)


try:
    user_site = site.getusersitepackages()
except Exception:
    user_site = None
if user_site:
    sys.path = [entry for entry in sys.path if Path(entry).resolve() != Path(user_site).resolve()]

import matplotlib.pyplot as plt
import rclpy
from geometry_msgs.msg import TransformStamped
from matplotlib.animation import FuncAnimation
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from phasespace_visualization_common import (
    DEFAULT_MAX_HEIGHT_M,
    RoomGeometry,
    PoseSample,
    compute_path_metrics,
    draw_room_2d,
    draw_room_3d,
    resolve_room_axes,
    resolve_room_center,
    room_bounds_from_center,
    set_axes_equal_3d,
    trajectory_arrays,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Live ROS 2 visualizer for PhaseSpace /phasespace TransformStamped data."
    )
    parser.add_argument(
        "--topic",
        default="/phasespace",
        help="TransformStamped topic to subscribe to.",
    )
    parser.add_argument(
        "--mode",
        choices=("2d", "3d"),
        default="2d",
        help="Live plot mode.",
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
        default=4000,
        help="Maximum number of samples retained per tracker.",
    )
    parser.add_argument(
        "--update-rate-hz",
        type=float,
        default=10.0,
        help="Figure refresh rate.",
    )
    parser.add_argument(
        "--room-anchor",
        choices=("trajectory_center", "center", "corner"),
        default="trajectory_center",
        help="How to place the room overlay.",
    )
    parser.add_argument(
        "--room-origin-x-m",
        type=float,
        default=0.0,
        help="Room x origin in meters when using center/corner anchoring.",
    )
    parser.add_argument(
        "--room-origin-z-m",
        type=float,
        default=0.0,
        help="Room z origin in meters when using center/corner anchoring.",
    )
    parser.add_argument(
        "--room-long-axis",
        choices=("auto", "x", "z"),
        default="auto",
        help="Assign the room's 12.468 m side to world x or z. Auto uses the larger observed span.",
    )
    parser.add_argument(
        "--room-height-m",
        type=float,
        default=DEFAULT_MAX_HEIGHT_M,
        help="Ceiling height shown in 3D mode.",
    )
    parser.add_argument(
        "--hide-room",
        action="store_true",
        help="Render trajectories without the room overlay.",
    )
    return parser.parse_args()


class PhaseSpaceLiveVisualizer(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("phasespace_live_visualizer")
        self.args = args
        self._lock = threading.Lock()
        self._samples: dict[str, deque[PoseSample]] = {}
        self.subscription = self.create_subscription(
            TransformStamped,
            args.topic,
            self._handle_message,
            qos_profile_sensor_data,
        )

    def _handle_message(self, message: TransformStamped) -> None:
        tracker_id = message.child_frame_id or "unknown"
        sample = PoseSample(
            stamp_ns=(message.header.stamp.sec * 10**9) + message.header.stamp.nanosec,
            tracker_id=tracker_id,
            x_m=message.transform.translation.x * self.args.position_scale,
            y_m=message.transform.translation.y * self.args.position_scale,
            z_m=message.transform.translation.z * self.args.position_scale,
            qw=message.transform.rotation.w,
            qx=message.transform.rotation.x,
            qy=message.transform.rotation.y,
            qz=message.transform.rotation.z,
        )
        with self._lock:
            samples = self._samples.setdefault(
                tracker_id, deque(maxlen=self.args.max_trail_points)
            )
            samples.append(sample)

    def snapshot(self) -> dict[str, list[PoseSample]]:
        with self._lock:
            return {
                tracker_id: list(samples)
                for tracker_id, samples in sorted(self._samples.items())
            }


def tracker_color(index: int):
    cmap = plt.get_cmap("tab10")
    return cmap(index % cmap.N)


def refresh_plot(frame_index: int, node: PhaseSpaceLiveVisualizer, args, figure, axis) -> None:
    del frame_index
    trajectories = node.snapshot()
    axis.clear()
    axis.set_facecolor("#faf8f4")

    if args.mode == "2d":
        axis.grid(color="#d5cdc0", linestyle=":", linewidth=0.8, alpha=0.7)
        axis.set_title("PhaseSpace Live View (2D Top-Down)")
        axis.set_xlabel("World x (m)")
        axis.set_ylabel("World z (m)")
    else:
        axis.set_title("PhaseSpace Live View (3D)")
        axis.set_xlabel("World x (m)")
        axis.set_ylabel("World z (m)")
        axis.set_zlabel("World y / height (m)")

    if not trajectories:
        if args.mode == "2d":
            axis.text(0.5, 0.5, "Waiting for /phasespace data...", ha="center", va="center", transform=axis.transAxes)
        else:
            axis.text2D(0.5, 0.5, "Waiting for /phasespace data...", ha="center", va="center", transform=axis.transAxes)
        figure.canvas.draw_idle()
        return

    all_x = []
    all_y = []
    all_z = []
    for samples in trajectories.values():
        _, xyz = trajectory_arrays(samples)
        if len(xyz):
            all_x.extend(xyz[:, 0].tolist())
            all_y.extend(xyz[:, 1].tolist())
            all_z.extend(xyz[:, 2].tolist())

    x_span_m = max(all_x) - min(all_x) if all_x else 0.0
    z_span_m = max(all_z) - min(all_z) if all_z else 0.0
    room = RoomGeometry(max_height_m=args.room_height_m)
    room_size_x_m, room_size_z_m, _ = resolve_room_axes(
        room,
        x_span_m=x_span_m,
        z_span_m=z_span_m,
        long_axis=args.room_long_axis,
    )
    room_center_x_m, room_center_z_m = resolve_room_center(
        trajectories,
        room_size_x_m=room_size_x_m,
        room_size_z_m=room_size_z_m,
        anchor=args.room_anchor,
        origin_x_m=args.room_origin_x_m,
        origin_z_m=args.room_origin_z_m,
    )

    if not args.hide_room:
        if args.mode == "2d":
            draw_room_2d(
                axis,
                center_x_m=room_center_x_m,
                center_z_m=room_center_z_m,
                size_x_m=room_size_x_m,
                size_z_m=room_size_z_m,
            )
        else:
            draw_room_3d(
                axis,
                center_x_m=room_center_x_m,
                center_z_m=room_center_z_m,
                size_x_m=room_size_x_m,
                size_z_m=room_size_z_m,
                max_height_m=room.max_height_m,
            )

    summary_lines = [
        f"trackers: {len(trajectories)}",
        f"room overlay: {'off' if args.hide_room else 'on'}",
    ]

    for index, (tracker_id, samples) in enumerate(trajectories.items()):
        elapsed_s, xyz = trajectory_arrays(samples)
        color = tracker_color(index)
        if args.mode == "2d":
            axis.plot(xyz[:, 0], xyz[:, 2], color=color, linewidth=2.2, alpha=0.95, label=f"Tracker {tracker_id}")
            axis.scatter([xyz[-1, 0]], [xyz[-1, 2]], color=[color], s=65, marker="o", edgecolors="black", linewidths=0.7)
        else:
            axis.plot(xyz[:, 0], xyz[:, 2], xyz[:, 1], color=color, linewidth=2.0, alpha=0.95)
            axis.scatter(
                [xyz[-1, 0]],
                [xyz[-1, 2]],
                [xyz[-1, 1]],
                color=[color],
                s=65,
                marker="o",
                edgecolors="black",
                linewidths=0.7,
                depthshade=False,
            )
        metrics = compute_path_metrics(samples)
        summary_lines.append(
            f"tracker {tracker_id}: {metrics['distance_m']:.2f} m over {metrics['duration_s']:.1f} s"
        )

    if args.mode == "2d":
        axis.legend(loc="upper right")
        xmin, xmax, zmin, zmax = room_bounds_from_center(
            room_center_x_m, room_center_z_m, room_size_x_m, room_size_z_m
        )
        if args.hide_room:
            xmin, xmax = min(all_x), max(all_x)
            zmin, zmax = min(all_z), max(all_z)
        pad_x = max(0.35, (xmax - xmin) * 0.05)
        pad_z = max(0.35, (zmax - zmin) * 0.05)
        axis.set_xlim(xmin - pad_x, xmax + pad_x)
        axis.set_ylim(zmin - pad_z, zmax + pad_z)
        axis.set_aspect("equal", adjustable="box")
        axis.text(
            0.02,
            0.98,
            "\n".join(summary_lines),
            transform=axis.transAxes,
            va="top",
            ha="left",
            family="monospace",
            fontsize=9,
            bbox={"boxstyle": "round,pad=0.45", "facecolor": "#faf7f2", "edgecolor": "#cabfae"},
        )
    else:
        xmin, xmax, zmin, zmax = room_bounds_from_center(
            room_center_x_m, room_center_z_m, room_size_x_m, room_size_z_m
        )
        ymin, ymax = 0.0, room.max_height_m
        if args.hide_room:
            xmin, xmax = min(all_x), max(all_x)
            zmin, zmax = min(all_z), max(all_z)
            ymin, ymax = 0.0, max(max(all_y), room.max_height_m)

        pad_x = max(0.35, (xmax - xmin) * 0.05)
        pad_z = max(0.35, (zmax - zmin) * 0.05)
        pad_y = max(0.25, (ymax - ymin) * 0.08)
        axis.set_xlim(xmin - pad_x, xmax + pad_x)
        axis.set_ylim(zmin - pad_z, zmax + pad_z)
        axis.set_zlim(max(0.0, ymin - pad_y), ymax + pad_y)
        set_axes_equal_3d(axis)
        axis.view_init(elev=25, azim=-58)
        axis.text2D(
            0.02,
            0.98,
            "\n".join(summary_lines),
            transform=axis.transAxes,
            va="top",
            ha="left",
            family="monospace",
            fontsize=9,
            bbox={"boxstyle": "round,pad=0.45", "facecolor": "#faf7f2", "edgecolor": "#cabfae"},
        )

    figure.canvas.draw_idle()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = PhaseSpaceLiveVisualizer(args)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    if args.mode == "2d":
        figure, axis = plt.subplots(figsize=(10.5, 8))
    else:
        figure = plt.figure(figsize=(11, 9))
        axis = figure.add_subplot(111, projection="3d")

    animation = FuncAnimation(
        figure,
        refresh_plot,
        interval=max(50, int(1000.0 / args.update_rate_hz)),
        fargs=(node, args, figure, axis),
        cache_frame_data=False,
    )
    figure._phasepace_animation = animation

    try:
        plt.show()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
