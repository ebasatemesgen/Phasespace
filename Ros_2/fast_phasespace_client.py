#!/usr/bin/env python3
import argparse
import time
from collections import deque

# Put this file in the same folder as phasespace_node_ros2.py
from phasespace_node_ros2 import Context, Type, load_tracker_configs


def create_server_trackers(owl: Context, jsonfiles):
    tracker_configs = load_tracker_configs(jsonfiles)
    for tracker_config in tracker_configs:
        tracker_id = tracker_config["tracker_id"]
        tracker_name = tracker_config["name"]
        owl.createTracker(tracker_id, "rigid", tracker_name)
        for marker in tracker_config["markers"]:
            owl.assignMarker(
                tracker_id,
                marker["id"],
                marker["name"],
                marker["options"],
            )
        print(
            f"[PhaseSpace] Loaded {tracker_config['source_label']} "
            f"as tracker id {tracker_id} with markers {tracker_config['marker_ids']}"
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="cs-phasespace.cs.umn.edu")
    parser.add_argument("--freq", type=float, default=960.0)
    parser.add_argument("--timeout", type=int, default=1000, help="microseconds")
    parser.add_argument("--stream", choices=["tcp", "udp"], default="udp")
    parser.add_argument(
        "--mode", choices=["markers", "rigids", "both"], default="rigids"
    )
    parser.add_argument("--jsonfiles", nargs="*", default=None)
    parser.add_argument("--use-server-trackers", action="store_true")
    args = parser.parse_args()

    owl = Context()

    # Open the PhaseSpace server connection.
    owl.open(args.device, "timeout=5000000")

    # Choose which event types you actually want.
    if args.mode == "markers":
        event_opts = "event.cameras=0 event.markers=1 event.rigids=0"
    elif args.mode == "rigids":
        event_opts = "event.cameras=0 event.markers=0 event.rigids=1"
    else:
        event_opts = "event.cameras=0 event.markers=1 event.rigids=1"

    owl.initialize(f"timeout=5000000 slave=0 {event_opts}")

    # Ask the server for the target rate.
    owl.frequency(args.freq)

    # Optional: wait briefly for the async frequency property to settle.
    t_deadline = time.perf_counter() + 2.0
    while time.perf_counter() < t_deadline:
        current = owl.frequency()
        if current is not None and abs(float(current) - args.freq) < 1e-3:
            break
        time.sleep(0.001)

    # Transport mode: 1 = TCP, 2 = UDP
    owl.streaming(2 if args.stream == "udp" else 1)

    # Create rigid trackers only if you need them and are not using ones
    # already configured on the server.
    if args.mode in ("rigids", "both") and (not args.use_server_trackers):
        if not args.jsonfiles:
            raise ValueError(
                "For rigid streaming, pass --use-server-trackers or --jsonfiles ..."
            )
        create_server_trackers(owl, args.jsonfiles)

    print(
        f"[PhaseSpace] Connected to {args.device} | "
        f"mode={args.mode} | stream={args.stream} | requested_freq={args.freq}"
    )

    # Keep only the freshest data locally.
    latest_markers = {}
    latest_rigids = {}

    # Simple rolling stats.
    recv_times = deque(maxlen=5000)
    n_frames = 0
    last_report = time.perf_counter()

    try:
        while owl.isOpen() and owl.property("initialized"):
            event = owl.nextEvent(args.timeout)
            if event is None:
                continue

            if event.type_id == Type.ERROR or getattr(event, "name", None) == "done":
                print("[PhaseSpace] Stream ended or server reported error.")
                break

            if event.type_id != Type.FRAME:
                continue

            now = time.perf_counter()
            recv_times.append(now)
            n_frames += 1

            if "markers" in event:
                for m in event.markers:
                    if m.cond > 0:
                        latest_markers[m.id] = {
                            "device_time": m.time,
                            "xyz": (m.x, m.y, m.z),
                            "recv_time": now,
                            "cond": m.cond,
                        }

            if "rigids" in event:
                for r in event.rigids:
                    if r.cond > 0:
                        latest_rigids[r.id] = {
                            "device_time": r.time,
                            "pose": tuple(r.pose),  # (x,y,z,w,x,y,z)
                            "recv_time": now,
                            "cond": r.cond,
                        }

            # Report once per second.
            if now - last_report >= 1.0:
                if len(recv_times) >= 2:
                    dt = recv_times[-1] - recv_times[0]
                    frame_rate = (len(recv_times) - 1) / dt if dt > 0 else 0.0
                else:
                    frame_rate = 0.0

                print(
                    f"[PhaseSpace] frame_events_per_sec={frame_rate:.1f} | "
                    f"markers={len(latest_markers)} | rigids={len(latest_rigids)}"
                )

                # Example: this is where your high-rate controller would run
                # directly from the freshest sample, not through ROS.
                # robot_pose = latest_rigids.get(1)
                # if robot_pose is not None:
                #     control_step(robot_pose)

                last_report = now

    finally:
        try:
            owl.done()
        except Exception:
            pass
        try:
            owl.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()