#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import os
import site
import sys
import tempfile
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
import numpy as np

from phasespace_visualization_common import (
    DEFAULT_MAX_HEIGHT_M,
    OPEN_LAB_SIDE_RECESS_M,
    RoomGeometry,
    bag_name_from_path,
    compute_path_metrics,
    draw_room_2d,
    draw_room_3d,
    global_xyz_bounds,
    load_transform_trajectories,
    resolve_room_axes,
    resolve_room_center,
    set_axes_equal_3d,
    trajectory_arrays,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Render 2D and 3D maps from a PhaseSpace rosbag2 MCAP recording."
    )
    parser.add_argument("bag_path", help="Path to a rosbag directory or .mcap file.")
    parser.add_argument(
        "--output-dir",
        default="visualization/outputs",
        help="Directory where rendered figures and summary files are written.",
    )
    parser.add_argument(
        "--mode",
        choices=("2d", "3d", "both"),
        default="both",
        help="Which visualizations to render.",
    )
    parser.add_argument(
        "--topic",
        default="/phasespace",
        help="TransformStamped topic to visualize.",
    )
    parser.add_argument(
        "--position-scale",
        type=float,
        default=0.001,
        help="Scale applied to recorded positions. 0.001 converts millimeters to meters.",
    )
    parser.add_argument(
        "--room-anchor",
        choices=("trajectory_center", "center", "corner"),
        default="trajectory_center",
        help="How to place the room footprint around the data.",
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
        help="Assign the room's 12.468 m side to world x or z. Auto picks the larger trajectory span.",
    )
    parser.add_argument(
        "--room-height-m",
        type=float,
        default=DEFAULT_MAX_HEIGHT_M,
        help="Ceiling height used for the 3D room wireframe.",
    )
    parser.add_argument(
        "--title",
        default=None,
        help="Optional title override for rendered figures.",
    )
    parser.add_argument(
        "--hide-room",
        action="store_true",
        help="Render trajectories without the room footprint/wireframe.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Open figures after rendering.",
    )
    return parser.parse_args()


def build_summary(
    trajectories: dict[str, list],
    room: RoomGeometry,
    room_size_x_m: float,
    room_size_z_m: float,
    room_center_x_m: float,
    room_center_z_m: float,
    room_long_axis: str,
    bag_name: str,
    bag_path: str,
    output_dir: Path,
    topic: str,
    position_scale: float,
) -> dict[str, object]:
    min_xyz, max_xyz = global_xyz_bounds(trajectories)
    tracker_metrics = {
        tracker_id: compute_path_metrics(samples)
        for tracker_id, samples in trajectories.items()
    }
    total_distance_m = sum(metric["distance_m"] for metric in tracker_metrics.values())
    max_duration_s = max(
        (metric["duration_s"] for metric in tracker_metrics.values()),
        default=0.0,
    )
    return {
        "bag_name": bag_name,
        "bag_path": str(Path(bag_path).resolve()),
        "output_dir": str(output_dir.resolve()),
        "topic": topic,
        "position_scale": position_scale,
        "tracker_ids": sorted(trajectories.keys()),
        "tracker_count": len(trajectories),
        "total_distance_m": total_distance_m,
        "max_duration_s": max_duration_s,
        "global_bounds_m": {
            "min": {"x": float(min_xyz[0]), "y": float(min_xyz[1]), "z": float(min_xyz[2])},
            "max": {"x": float(max_xyz[0]), "y": float(max_xyz[1]), "z": float(max_xyz[2])},
        },
        "room_model": {
            "width_m": room.width_m,
            "length_m": room.length_m,
            "side_recess_m": room.side_recess_m,
            "room_size_x_m": room_size_x_m,
            "room_size_z_m": room_size_z_m,
            "center_x_m": room_center_x_m,
            "center_z_m": room_center_z_m,
            "long_axis": room_long_axis,
            "max_height_m": room.max_height_m,
        },
        "trackers": tracker_metrics,
    }


def save_samples_csv(trajectories: dict[str, list], output_path: Path) -> None:
    with output_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "tracker_id",
                "stamp_ns",
                "elapsed_s",
                "x_m",
                "y_m",
                "z_m",
                "qw",
                "qx",
                "qy",
                "qz",
            ]
        )
        for tracker_id, samples in trajectories.items():
            elapsed_s, xyz = trajectory_arrays(samples)
            for index, sample in enumerate(samples):
                writer.writerow(
                    [
                        tracker_id,
                        sample.stamp_ns,
                        float(elapsed_s[index]),
                        float(xyz[index, 0]),
                        float(xyz[index, 1]),
                        float(xyz[index, 2]),
                        sample.qw,
                        sample.qx,
                        sample.qy,
                        sample.qz,
                    ]
                )


def tracker_color(index: int):
    cmap = plt.get_cmap("tab10")
    return cmap(index % cmap.N)


def render_2d_map(
    trajectories: dict[str, list],
    output_path: Path,
    title: str,
    room: RoomGeometry,
    room_size_x_m: float,
    room_size_z_m: float,
    room_center_x_m: float,
    room_center_z_m: float,
    room_long_axis: str,
    show_room: bool,
) -> plt.Figure:
    fig = plt.figure(figsize=(14, 8), layout="constrained")
    grid = fig.add_gridspec(2, 2, width_ratios=(3.2, 1.3), height_ratios=(1.7, 1.0))
    ax_map = fig.add_subplot(grid[:, 0])
    ax_height = fig.add_subplot(grid[0, 1])
    ax_text = fig.add_subplot(grid[1, 1])

    ax_text.axis("off")
    ax_map.set_facecolor("#f8f5ef")
    ax_height.set_facecolor("#fbfaf6")
    ax_map.grid(color="#c8c1b6", linestyle=":", linewidth=0.8, alpha=0.65)
    ax_height.grid(color="#d9d1c3", linestyle=":", linewidth=0.8, alpha=0.65)

    if show_room:
        room_limits = draw_room_2d(
            ax_map,
            center_x_m=room_center_x_m,
            center_z_m=room_center_z_m,
            size_x_m=room_size_x_m,
            size_z_m=room_size_z_m,
        )
    else:
        room_limits = None

    scatter_handle = None
    legend_handles = []
    summary_lines = [
        f"Room footprint from README: {room.width_m:.3f} m x {room.length_m:.3f} m",
        f"3D clearance modeled to {room.max_height_m:.1f} m",
        f"Side recess noted in floor plan: {OPEN_LAB_SIDE_RECESS_M:.3f} m",
        f"Long room axis mapped to world {room_long_axis}",
        "",
    ]

    for index, (tracker_id, samples) in enumerate(trajectories.items()):
        elapsed_s, xyz = trajectory_arrays(samples)
        color = tracker_color(index)
        sample_stride = max(1, len(samples) // 70)
        ax_map.plot(
            xyz[:, 0],
            xyz[:, 2],
            color=color,
            linewidth=2.4,
            alpha=0.9,
            zorder=2,
        )
        scatter_handle = ax_map.scatter(
            xyz[::sample_stride, 0],
            xyz[::sample_stride, 2],
            c=elapsed_s[::sample_stride],
            cmap="viridis",
            s=26,
            alpha=0.9,
            edgecolors="none",
            zorder=3,
        )
        ax_map.scatter(
            xyz[0, 0],
            xyz[0, 2],
            s=90,
            marker="o",
            facecolor="white",
            edgecolor=color,
            linewidth=2.0,
            zorder=4,
        )
        ax_map.scatter(
            xyz[-1, 0],
            xyz[-1, 2],
            s=120,
            marker="X",
            facecolor=color,
            edgecolor="black",
            linewidth=0.8,
            zorder=5,
        )

        ax_height.plot(
            elapsed_s,
            xyz[:, 1],
            color=color,
            linewidth=2.0,
            alpha=0.9,
        )
        ax_height.scatter(
            elapsed_s[::sample_stride],
            xyz[::sample_stride, 1],
            c=elapsed_s[::sample_stride],
            cmap="viridis",
            s=22,
            alpha=0.75,
            edgecolors="none",
        )

        metrics = compute_path_metrics(samples)
        summary_lines.extend(
            [
                f"Tracker {tracker_id}",
                f"  samples: {int(metrics['sample_count'])}",
                f"  duration: {metrics['duration_s']:.2f} s",
                f"  distance: {metrics['distance_m']:.2f} m",
                f"  max speed: {metrics['max_speed_mps']:.2f} m/s",
                f"  height range: {metrics['min_height_m']:.2f} to {metrics['max_height_m']:.2f} m",
                "",
            ]
        )
        legend_handles.append(
            plt.Line2D([0], [0], color=color, linewidth=2.4, label=f"Tracker {tracker_id}")
        )

    if scatter_handle is not None:
        colorbar = fig.colorbar(scatter_handle, ax=[ax_map, ax_height], pad=0.02, shrink=0.88)
        colorbar.set_label("Elapsed time (s)")

    ax_map.set_title(title)
    ax_map.set_xlabel("World x (m)")
    ax_map.set_ylabel("World z (m)")
    ax_map.set_aspect("equal", adjustable="box")
    ax_map.legend(handles=legend_handles, loc="upper right", frameon=True)

    ax_height.set_title("Height vs. Time")
    ax_height.set_xlabel("Elapsed time (s)")
    ax_height.set_ylabel("World y / height (m)")

    ax_text.text(
        0.0,
        1.0,
        "\n".join(summary_lines).strip(),
        va="top",
        ha="left",
        family="monospace",
        fontsize=10,
        bbox={"boxstyle": "round,pad=0.5", "facecolor": "#faf7f2", "edgecolor": "#cabfae"},
    )

    min_xyz, max_xyz = global_xyz_bounds(trajectories)
    if room_limits:
        xmin, xmax, zmin, zmax = room_limits
    else:
        xmin, xmax = float(min_xyz[0]), float(max_xyz[0])
        zmin, zmax = float(min_xyz[2]), float(max_xyz[2])

    pad_x = max(0.35, (xmax - xmin) * 0.05)
    pad_z = max(0.35, (zmax - zmin) * 0.05)
    ax_map.set_xlim(xmin - pad_x, xmax + pad_x)
    ax_map.set_ylim(zmin - pad_z, zmax + pad_z)

    fig.savefig(output_path, dpi=220, bbox_inches="tight")
    return fig


def render_3d_map(
    trajectories: dict[str, list],
    output_path: Path,
    title: str,
    room: RoomGeometry,
    room_size_x_m: float,
    room_size_z_m: float,
    room_center_x_m: float,
    room_center_z_m: float,
    room_long_axis: str,
    show_room: bool,
) -> plt.Figure:
    fig = plt.figure(figsize=(12, 9), layout="constrained")
    ax = fig.add_subplot(111, projection="3d")
    ax.set_facecolor("#faf8f4")

    room_limits = None
    if show_room:
        room_limits = draw_room_3d(
            ax,
            center_x_m=room_center_x_m,
            center_z_m=room_center_z_m,
            size_x_m=room_size_x_m,
            size_z_m=room_size_z_m,
            max_height_m=room.max_height_m,
        )

    scatter_handle = None
    summary_lines = [
        "README room model",
        f"  footprint: {room.width_m:.3f} m x {room.length_m:.3f} m",
        f"  long axis mapped to world {room_long_axis}",
        f"  recess note: {OPEN_LAB_SIDE_RECESS_M:.3f} m side inset",
        "",
    ]

    for index, (tracker_id, samples) in enumerate(trajectories.items()):
        elapsed_s, xyz = trajectory_arrays(samples)
        color = tracker_color(index)
        sample_stride = max(1, len(samples) // 90)

        ax.plot(
            xyz[:, 0],
            xyz[:, 2],
            xyz[:, 1],
            color=color,
            linewidth=2.0,
            alpha=0.9,
        )
        ax.plot(
            xyz[:, 0],
            xyz[:, 2],
            np.zeros(len(samples)),
            color=color,
            linewidth=1.0,
            linestyle=":",
            alpha=0.35,
        )
        scatter_handle = ax.scatter(
            xyz[::sample_stride, 0],
            xyz[::sample_stride, 2],
            xyz[::sample_stride, 1],
            c=elapsed_s[::sample_stride],
            cmap="viridis",
            s=24,
            depthshade=False,
            alpha=0.9,
        )
        ax.scatter(
            [xyz[0, 0]],
            [xyz[0, 2]],
            [xyz[0, 1]],
            s=90,
            marker="o",
            facecolor="white",
            edgecolor=color,
            linewidth=2.0,
            depthshade=False,
        )
        ax.scatter(
            [xyz[-1, 0]],
            [xyz[-1, 2]],
            [xyz[-1, 1]],
            s=120,
            marker="X",
            facecolor=color,
            edgecolor="black",
            linewidth=0.8,
            depthshade=False,
        )

        metrics = compute_path_metrics(samples)
        summary_lines.extend(
            [
                f"Tracker {tracker_id}",
                f"  duration: {metrics['duration_s']:.2f} s",
                f"  distance: {metrics['distance_m']:.2f} m",
                f"  peak speed: {metrics['max_speed_mps']:.2f} m/s",
                "",
            ]
        )

    if scatter_handle is not None:
        colorbar = fig.colorbar(scatter_handle, ax=ax, shrink=0.72, pad=0.02)
        colorbar.set_label("Elapsed time (s)")

    min_xyz, max_xyz = global_xyz_bounds(trajectories)
    if room_limits:
        xmin, xmax, zmin, zmax = room_limits
        ymin, ymax = 0.0, room.max_height_m
    else:
        xmin, xmax = float(min_xyz[0]), float(max_xyz[0])
        zmin, zmax = float(min_xyz[2]), float(max_xyz[2])
        ymin, ymax = 0.0, max(room.max_height_m, float(max_xyz[1]) + 0.25)

    pad_x = max(0.4, (xmax - xmin) * 0.05)
    pad_z = max(0.4, (zmax - zmin) * 0.05)
    pad_y = max(0.3, (ymax - ymin) * 0.08)
    ax.set_xlim(xmin - pad_x, xmax + pad_x)
    ax.set_ylim(zmin - pad_z, zmax + pad_z)
    ax.set_zlim(max(0.0, ymin - pad_y), ymax + pad_y)
    set_axes_equal_3d(ax)

    ax.set_title(title)
    ax.set_xlabel("World x (m)")
    ax.set_ylabel("World z (m)")
    ax.set_zlabel("World y / height (m)")
    ax.view_init(elev=26, azim=-58)
    ax.text2D(
        0.02,
        0.98,
        "\n".join(summary_lines).strip(),
        transform=ax.transAxes,
        va="top",
        ha="left",
        family="monospace",
        fontsize=10,
        bbox={"boxstyle": "round,pad=0.45", "facecolor": "#faf7f2", "edgecolor": "#cabfae"},
    )

    fig.savefig(output_path, dpi=220, bbox_inches="tight")
    return fig


def main() -> int:
    args = parse_args()
    bag_name = bag_name_from_path(args.bag_path)
    output_dir = Path(args.output_dir) / bag_name
    output_dir.mkdir(parents=True, exist_ok=True)

    room = RoomGeometry(max_height_m=args.room_height_m)
    trajectories = load_transform_trajectories(
        args.bag_path,
        topic=args.topic,
        position_scale=args.position_scale,
    )
    if not trajectories:
        raise RuntimeError(f"No samples found on topic {args.topic!r} in {args.bag_path}")

    min_xyz, max_xyz = global_xyz_bounds(trajectories)
    room_size_x_m, room_size_z_m, room_long_axis = resolve_room_axes(
        room,
        x_span_m=float(max_xyz[0] - min_xyz[0]),
        z_span_m=float(max_xyz[2] - min_xyz[2]),
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

    summary = build_summary(
        trajectories=trajectories,
        room=room,
        room_size_x_m=room_size_x_m,
        room_size_z_m=room_size_z_m,
        room_center_x_m=room_center_x_m,
        room_center_z_m=room_center_z_m,
        room_long_axis=room_long_axis,
        bag_name=bag_name,
        bag_path=args.bag_path,
        output_dir=output_dir,
        topic=args.topic,
        position_scale=args.position_scale,
    )
    with (output_dir / "summary.json").open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)
    save_samples_csv(trajectories, output_dir / "trajectory_samples.csv")

    title = args.title or f"PhaseSpace Trajectory Map: {bag_name}"
    figures = []
    if args.mode in {"2d", "both"}:
        figures.append(
            render_2d_map(
            trajectories=trajectories,
            output_path=output_dir / "trajectory_map_2d.png",
            title=title,
            room=room,
            room_size_x_m=room_size_x_m,
            room_size_z_m=room_size_z_m,
            room_center_x_m=room_center_x_m,
            room_center_z_m=room_center_z_m,
            room_long_axis=room_long_axis,
            show_room=not args.hide_room,
        )
        )
    if args.mode in {"3d", "both"}:
        figures.append(
            render_3d_map(
            trajectories=trajectories,
            output_path=output_dir / "trajectory_map_3d.png",
            title=title,
            room=room,
            room_size_x_m=room_size_x_m,
            room_size_z_m=room_size_z_m,
            room_center_x_m=room_center_x_m,
            room_center_z_m=room_center_z_m,
            room_long_axis=room_long_axis,
            show_room=not args.hide_room,
        )
        )

    if args.show:
        plt.show()
    else:
        for figure in figures:
            plt.close(figure)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
