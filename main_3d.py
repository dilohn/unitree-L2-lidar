import socket
import struct
import time
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from decode_lidar_3d import decode_packet

# Connection information. These are all defaults
LOCAL_IP = "192.168.1.2"
LOCAL_PORT = 6201
REMOTE_IP = "192.168.1.62"
REMOTE_PORT = 6101

# How long to keep points
PERSIST = 0.5


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((LOCAL_IP, LOCAL_PORT))
    print(f"Listening on {LOCAL_IP}:{LOCAL_PORT} for packets from {REMOTE_IP}:{REMOTE_PORT}")

    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')
    ax.set_title('Real-time 3D LiDAR Scan')

    # History buffer of (x, y, z, intensity, timestamp)
    pts_history = []

    try:
        while True:
            data, addr = sock.recvfrom(65535)
            if addr[0] != REMOTE_IP or addr[1] != REMOTE_PORT:
                continue

            # Reconstruct UDP header for decoder
            udp_len = len(data) + 8
            udp_header = struct.pack("!HHHH",
                                    REMOTE_PORT,
                                    LOCAL_PORT,
                                    udp_len,
                                    0)
            raw_packet = udp_header + data

            try:
                decoded = decode_packet(raw_packet)
            except Exception as e:
                print(f"[Error] decode failed: {e}")
                continue

            geom = decoded['geometry']
            calib = decoded['calib_param']
            ranges = [pt['distance_mm'] for pt in decoded['points']]
            intensities = [pt['intensity'] for pt in decoded['points']]
            n = decoded['point_num']

            # Precompute rotation biases
            beta = calib['beta_angle']
            xi = calib['xi_angle']
            sin_beta = math.sin(beta)
            cos_beta = math.cos(beta)
            sin_xi = math.sin(xi)
            cos_xi = math.cos(xi)
            cos_beta_sin_xi = cos_beta * sin_xi
            sin_beta_cos_xi = sin_beta * cos_xi
            sin_beta_sin_xi = sin_beta * sin_xi
            cos_beta_cos_xi = cos_beta * cos_xi

            # Angle iterators
            alpha_cur = geom['angle_min'] + calib['alpha_angle_bias']
            alpha_step = geom['angle_increment']
            theta_cur = geom['h_angle_start'] + calib['theta_angle_bias']
            theta_step = geom['h_angle_step']
            now = time.time()

            # Convert to 3D points
            for j in range(n):
                raw_r = ranges[j]
                if raw_r < 1:
                    alpha_cur += alpha_step
                    theta_cur += theta_step
                    continue

                # Apply scale and bias
                r = calib['range_scale'] * (raw_r + calib['range_bias'])

                # Spherical to Cartesian with calibration
                sin_a = math.sin(alpha_cur)
                cos_a = math.cos(alpha_cur)
                sin_t = math.sin(theta_cur)
                cos_t = math.cos(theta_cur)

                A = (-cos_beta_sin_xi + sin_beta_cos_xi * sin_a) * r + calib['b_axis_dist']
                B = cos_a * cos_xi * r
                C = (sin_beta_sin_xi + cos_beta_cos_xi * sin_a) * r

                x = cos_t * A - sin_t * B
                y = sin_t * A + cos_t * B
                z = C + calib['a_axis_dist']

                pts_history.append((x, y, z, intensities[j], now))

                alpha_cur += alpha_step
                theta_cur += theta_step

            # Cull old points
            pts_history = [pt for pt in pts_history if now - pt[4] <= PERSIST]

            # Extract coordinates and colors
            if pts_history:
                xs, ys, zs, cs, ts = zip(*pts_history)
            else:
                xs = ys = zs = cs = []

            ax.clear()
            ax.scatter(xs, ys, zs, s=1, c=cs, cmap='gray', vmin=0, vmax=255)
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.set_title('3D LiDAR Scan')
            plt.pause(0.001)

    except KeyboardInterrupt:
        print("Interrupted")
    finally:
        sock.close()

if __name__ == '__main__':
    main()
