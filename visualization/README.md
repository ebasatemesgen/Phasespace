# PhaseSpace Visualization

This folder adds a separate visualization workflow for your recorded `/phasespace` data without changing the existing capture node in [Ros_2/phasespace_node_ros2.py](/home/eba/Desktop/ICRA_2027/Phasespace/Ros_2/phasespace_node_ros2.py).

It includes:

- `phasespace_bag_visualizer.py`: reads a rosbag2 MCAP directly and renders polished 2D and 3D maps.
- `phasespace_live_visualizer_ros2.py`: subscribes to a live ROS 2 `/phasespace` topic and shows the path in 2D or 3D.
- `phasespace_live_visualizer_web.py`: serves a low-latency browser view that renders at display rate and can ingest either ROS 2 `/phasespace` data or a direct PhaseSpace stream.
- `phasespace_visualization_common.py`: shared bag parsing, room geometry, and plotting helpers.

## Fast live visualization

For real-time use, prefer the browser visualizer instead of the Matplotlib live viewer.

Why:

- The browser path keeps rendering on `requestAnimationFrame`, so the canvas can stay smooth at `60 FPS`.
- The Python bridge only forwards fresh samples and keeps tracker trails bounded.
- Direct mode can bypass ROS 2 entirely and pull PhaseSpace data over `UDP`, which reduces latency.

### ROS 2 input

Terminal 1, publish `/phasespace` at a higher rate than the old `10 Hz` example:

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 Ros_2/phasespace_node_ros2.py \
  --device=cs-phasespace.cs.umn.edu \
  --stream=udp \
  --freq=120 \
  --jsonfiles 106A-ebasa_tracker.json
```

Terminal 2, launch the browser visualizer:

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 visualization/phasespace_live_visualizer_web.py \
  --source ros2 \
  --topic /phasespace
```

Then open:

- `http://127.0.0.1:8765/`

### Direct PhaseSpace input, no ROS 2

```bash
/usr/bin/python3 visualization/phasespace_live_visualizer_web.py \
  --source direct \
  --device cs-phasespace.cs.umn.edu \
  --stream udp \
  --freq 120 \
  --jsonfiles 106A-ebasa_tracker.json
```

If the tracker definitions are already active on the PhaseSpace server, use:

```bash
/usr/bin/python3 visualization/phasespace_live_visualizer_web.py \
  --source direct \
  --device cs-phasespace.cs.umn.edu \
  --stream udp \
  --freq 120 \
  --use-server-trackers
```

Useful flags:

- `--port <port>`
- `--max-trail-points <count>`
- `--hide-room`
- `--room-anchor trajectory_center|center|corner`
- `--room-origin-x-m <value>`
- `--room-origin-z-m <value>`
- `--room-long-axis auto|x|z`

## Room model

The room geometry comes from the floor plan in [README.md](/home/eba/Desktop/ICRA_2027/Phasespace/README.md):

- Main open-lab footprint: `6.391 m x 12.468 m`
- Side recess called out in the plan: `0.651 m`
- Height modeled for 3D views: up to `3.0 m`

Important note:

- The Phasespace world origin is user-defined during alignment, so the exact wall placement relative to `(x, z) = (0, 0)` is not fixed by the bag alone.
- The scripts therefore let you choose how the room is anchored.
- By default, the offline renderer centers the room around the trajectory and auto-assigns the room's long side to whichever world axis spans more distance.

## Offline rosbag rendering

Use `/usr/bin/python3` with the ROS environment sourced:

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 visualization/phasespace_bag_visualizer.py \
  rosbag2_2026_03_10-17_28_50 \
  --mode both
```

Outputs are written to:

- [visualization/outputs](/home/eba/Desktop/ICRA_2027/Phasespace/visualization/outputs)

For each bag, the renderer saves:

- `trajectory_map_2d.png`
- `trajectory_map_3d.png`
- `trajectory_samples.csv`
- `summary.json`

Useful options:

- `--room-anchor trajectory_center|center|corner`
- `--room-origin-x-m <value>`
- `--room-origin-z-m <value>`
- `--room-long-axis auto|x|z`
- `--hide-room`
- `--show`

## Live ROS 2 visualization

Run the viewer in one terminal:

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 visualization/phasespace_live_visualizer_ros2.py --mode 2d
```

Or 3D:

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 visualization/phasespace_live_visualizer_ros2.py --mode 3d
```

Then publish or replay `/phasespace` data from another terminal.

Important note:

- `phasespace_live_visualizer_ros2.py` uses Matplotlib and is still useful for quick inspection.
- It is not the recommended path when you want low-latency live viewing.
- Use `phasespace_live_visualizer_web.py` for the fast path.

## Notes on this bag

For [rosbag2_2026_03_10-17_28_50](/home/eba/Desktop/ICRA_2027/Phasespace/rosbag2_2026_03_10-17_28_50), the recorded `/phasespace` topic contains:

- `541` `TransformStamped` messages
- one tracker with `child_frame_id = "2"`
- positions that fit the room dimensions when interpreted as millimeters and converted to meters

The `/phasespace_vel` topic exists too, but its stored linear velocities are zero in this bag, so the visualizer computes motion statistics from successive pose samples instead.
