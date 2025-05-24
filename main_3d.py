import socket
import signal
import sys
import numpy as np
import threading
import time
import open3d as o3d

from decode_lidar_3d import decode_packet, _print_summary

LOCAL_IP = "192.168.1.2"
LOCAL_PORT = 6201
REMOTE_IP = "192.168.1.62"
REMOTE_PORT = 6101
MAX_SIZE = 65535

POINT_SIZE = 2
INITIAL_CAPACITY = 10000
VOXEL_SIZE = 0.05
UPDATE_INTERVAL = 0.1
BACKGROUND_COLOR = [0.1, 0.1, 0.1]
PERSISTENCE = 1.0

points_lock = threading.Lock()
point_cloud = o3d.geometry.PointCloud()
point_timestamps = []
vis = None
running = True
new_points_available = False

def _shutdown(signum, frame):
    global running
    print("Shutting down...")
    running = False
    if vis is not None:
        vis.destroy_window()
    sys.exit(0)

def add_points(points):
    global point_cloud, point_timestamps, new_points_available

    if len(points) == 0:
        return

    new_cloud = o3d.geometry.PointCloud()
    new_cloud.points = o3d.utility.Vector3dVector(points[:, :3])

    intensities = points[:, 3]
    if len(intensities) > 0:
        min_i = np.min(intensities)
        max_i = np.max(intensities)
        if max_i > min_i:
            normalized = (intensities - min_i) / (max_i - min_i)
        else:
            normalized = np.ones_like(intensities) * 0.5

        colors = np.zeros((len(normalized), 3))
        colors[:, 0] = 0.4 * normalized
        colors[:, 1] = 0.4 + 0.6 * normalized
        colors[:, 2] = 0.7 + 0.3 * normalized
        new_cloud.colors = o3d.utility.Vector3dVector(colors)

    with points_lock:
        current_time = time.time()
        if PERSISTENCE > 0:
            point_timestamps.extend([current_time] * len(new_cloud.points))
        point_cloud += new_cloud
        new_points_available = True

def remove_old_points():
    global point_cloud, point_timestamps, new_points_available

    if PERSISTENCE <= 0 or not point_timestamps:
        return False

    current_time = time.time()
    cutoff = current_time - PERSISTENCE

    keep_indices = [i for i, ts in enumerate(point_timestamps) if ts >= cutoff]
    if len(keep_indices) == len(point_timestamps):
        return False

    if not keep_indices:
        point_cloud.clear()
        point_timestamps.clear()
        print(f"Removed all points (persistence: {PERSISTENCE}s)")
        return True

    pts = np.asarray(point_cloud.points)
    cols = np.asarray(point_cloud.colors)
    new_pts = pts[keep_indices]
    new_cols = cols[keep_indices]
    point_cloud.points = o3d.utility.Vector3dVector(new_pts)
    point_cloud.colors = o3d.utility.Vector3dVector(new_cols)
    point_timestamps[:] = [point_timestamps[i] for i in keep_indices]

    removed = len(pts) - len(new_pts)
    if removed > 0:
        print(f"Removed {removed} old points (persistence: {PERSISTENCE}s)")
    return True

def visualization_thread():
    global vis, point_cloud, running, new_points_available

    # Use the VisualizerWithKeyCallback variant:
    vis = o3d.visualization.VisualizerWithKeyCallback()
    title = "LiDAR Point Cloud Visualization"
    if PERSISTENCE > 0:
        title += f" (Persistence: {PERSISTENCE}s)"
    vis.create_window(title, width=1024, height=768)

    opt = vis.get_render_option()
    opt.background_color = np.asarray(BACKGROUND_COLOR)
    opt.point_size = POINT_SIZE

    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=1.0, origin=[0, 0, 0]
    )
    vis.add_geometry(coord_frame)
    vis.add_geometry(point_cloud)

    ctr = vis.get_view_control()
    ctr.set_zoom(0.8)
    ctr.set_front([0, 0, -1])
    ctr.set_lookat([0, 0, 0])
    ctr.set_up([0, -1, 0])

    print("Visualization ready. Controls:")
    print("  - Left-click + drag: Rotate")
    print("  - Right-click + drag: Pan")
    print("  - Mouse wheel: Zoom in/out")
    print("  - Shift + left-click: Rotate around scene point")
    print("  - 'r' key: Reset view")
    print("  - 'c' or 'C' key: Clear all points")
    print("  - 'h' key: Display help message")
    print("  - '-/+' keys: Decrease/increase point size")

    def clear_callback(vis):
        global point_cloud, point_timestamps, new_points_available
        with points_lock:
            point_cloud.clear()
            point_timestamps.clear()
            new_points_available = True
            print("Cleared all points")
        return True

    # Register for both lowercase and uppercase C:
    vis.register_key_callback(ord('c'), clear_callback)
    vis.register_key_callback(ord('C'), clear_callback)

    last_update = time.time()
    last_persist = time.time()
    point_count = 0

    while running:
        now = time.time()
        updated = False

        if PERSISTENCE > 0 and (now - last_persist) > UPDATE_INTERVAL:
            with points_lock:
                if remove_old_points():
                    updated = True
            last_persist = now

        if new_points_available and (now - last_update) > UPDATE_INTERVAL:
            with points_lock:
                vis.update_geometry(point_cloud)
                new_points_available = False
                updated = True
                if len(point_cloud.points) != point_count:
                    point_count = len(point_cloud.points)
                    print(f"Point cloud size: {point_count} points")
            last_update = now

        if not vis.poll_events():
            running = False
            break

        if updated:
            vis.update_renderer()

        time.sleep(0.01)

    vis.destroy_window()

def udp_receiver_thread():
    global running

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((LOCAL_IP, LOCAL_PORT))
    except Exception as e:
        print(f"Failed to bind to {LOCAL_IP}:{LOCAL_PORT} — {e}", file=sys.stderr)
        running = False
        return

    print(f"Listening on {LOCAL_IP}:{LOCAL_PORT}, expecting packets from {REMOTE_IP}:{REMOTE_PORT}")
    sock.setblocking(False)

    while running:
        try:
            data, addr = sock.recvfrom(MAX_SIZE)
            if addr == (REMOTE_IP, REMOTE_PORT):
                if len(data) == 1044:
                    try:
                        decoded = decode_packet(data)
                        add_points(decoded["points"])
                    except Exception as e:
                        print(f"[Error] Failed to decode packet: {e}", file=sys.stderr)
                else:
                    print(
                        f"[Warning] Received packet of unexpected size {len(data)} bytes",
                        file=sys.stderr,
                    )
        except BlockingIOError:
            time.sleep(0.001)
        except Exception as e:
            if running:
                print(f"[Error] {e}", file=sys.stderr)
            time.sleep(0.001)

def main():
    signal.signal(signal.SIGINT, _shutdown)

    vis_thread = threading.Thread(target=visualization_thread)
    vis_thread.daemon = True
    vis_thread.start()

    udp_thread = threading.Thread(target=udp_receiver_thread)
    udp_thread.daemon = True
    udp_thread.start()

    try:
        while running and vis_thread.is_alive() and udp_thread.is_alive():
            time.sleep(0.1)
    except KeyboardInterrupt:
        _shutdown(None, None)

if __name__ == "__main__":
    main()
