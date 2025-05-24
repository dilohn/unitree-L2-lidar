import socket
import struct
import time
import math
import pickle

from decode_lidar_3d import decode_packet

# How many minutes to capture
DURATION_MINUTES = 0.5

# Where to write the pickle
OUTPUT_FILE = "lidar_capture.pkl"

LOCAL_IP = "192.168.1.2"
LOCAL_PORT = 6201
REMOTE_IP = "192.168.1.62"
REMOTE_PORT = 6101

# Filter for range of view
MIN_DEG = 45.0
MAX_DEG = 180.0


def main(duration_min, output_file):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((LOCAL_IP, LOCAL_PORT))
    print(f"Capturing LiDAR for {duration_min:.1f} min to {output_file}")

    end_time = time.time() + duration_min * 60.0
    records = []

    try:
        while time.time() < end_time:
            data, addr = sock.recvfrom(65535)
            if addr[0] != REMOTE_IP or addr[1] != REMOTE_PORT:
                continue

            # Rebuild an 8-byte UDP header for our decoder
            udp_header = struct.pack("!HHHH",
                                     REMOTE_PORT,
                                     LOCAL_PORT,
                                     len(data) + 8,
                                     0)
            raw = udp_header + data

            try:
                decoded = decode_packet(raw)
            except Exception as e:
                print(f"[Warning] decode failed: {e}")
                continue

            # Geometry and calibration
            geom = decoded['geometry']
            calib = decoded['calib_param']
            beta, xi = calib['beta_angle'], calib['xi_angle']
            sin_b, cos_b = math.sin(beta), math.cos(beta)
            sin_x, cos_x = math.sin(xi), math.cos(xi)
            cbs_x = cos_b * sin_x
            sbc_x = sin_b * cos_x
            sbs_x = sin_b * sin_x
            cbc_x = cos_b * cos_x

            # Starting angles for scan
            alpha = geom['angle_min'] + calib['alpha_angle_bias']
            theta = geom['h_angle_start'] + calib['theta_angle_bias']
            a_step, t_step = geom['angle_increment'], geom['h_angle_step']

            pts = []
            ts = time.time()

            # Decode each point and filter by actual world azimuth
            for pt in decoded['points']:
                r_raw = pt['distance_mm']
                if r_raw < 1:
                    # Advance angles for every point
                    alpha += a_step; theta += t_step
                    continue

                # Compute range with bias/scale
                r = calib['range_scale'] * (r_raw + calib['range_bias'])

                # Get angles for this point
                ang_a = alpha
                ang_t = theta
                alpha += a_step; theta += t_step

                sa, ca = math.sin(ang_a), math.cos(ang_a)
                st, ct = math.sin(ang_t), math.cos(ang_t)

                # Apply calibration transformation
                A = (-cbs_x + sbc_x * sa) * r + calib['b_axis_dist']
                B = ca * cos_x * r
                C = (sbs_x + cbc_x * sa) * r

                # Cartesian coordinates
                x = ct * A - st * B
                y = st * A + ct * B
                z = C + calib['a_axis_dist']
                intensity = pt['intensity'] / 255.0

                # World azimuth = angle from x-axis in XY plane
                az_deg = math.degrees(math.atan2(y, x))
                if az_deg < 0:
                    az_deg += 360.0
                # Filter based on degrees
                if az_deg < MIN_DEG or az_deg > MAX_DEG:
                    continue

                pts.append((x, y, z, intensity))

            records.append({'timestamp': ts, 'points': pts})

    except KeyboardInterrupt:
        print("Interrupted")
    finally:
        sock.close()
        with open(output_file, 'wb') as f:
            pickle.dump(records, f)
        print(f"Saved {len(records)} frames to {output_file}")


if __name__ == "__main__":
    main(DURATION_MINUTES, OUTPUT_FILE)
