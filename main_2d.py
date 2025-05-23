import socket
import struct
import matplotlib.pyplot as plt
import numpy as np

LOCAL_IP = '192.168.1.2'
LOCAL_PORT = 6201
REMOTE_IP = '192.168.1.62'
REMOTE_PORT = 6101

HEADER_MAGIC = b'\x55\xAA\x05\x0A'
POINT2D_TYPE = 103

xs_all = []
ys_all = []
int_all = []

R_LIMIT = None

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((LOCAL_IP, LOCAL_PORT))

plt.ion()
fig, ax = plt.subplots()
sc = ax.scatter([], [], s=1, c=[], cmap='gray', alpha=0.7)
ax.set_aspect('equal')
plt.show()

buffer = bytearray()

while True:
    data, addr = sock.recvfrom(2048)
    if addr != (REMOTE_IP, REMOTE_PORT):
        continue
    buffer.extend(data)

    while True:
        if len(buffer) < 12:
            break
        if buffer[0:4] != HEADER_MAGIC:
            idx = buffer.find(HEADER_MAGIC, 1)
            if idx == -1:
                buffer.clear()
                break
            del buffer[:idx]
            continue

        pkt_type, pkt_size = struct.unpack_from('<II', buffer, 4)
        if len(buffer) < pkt_size:
            break

        packet = bytes(buffer[:pkt_size])
        del buffer[:pkt_size]

        if pkt_type != POINT2D_TYPE:
            continue

        off = 12
        _, _, sec, nsec = struct.unpack_from('<IIII', packet, off)
        off += 16
        off += 2*4 + 7*4
        off += 8*4
        scan_period, _, r_max, a_min, a_inc, _ = struct.unpack_from('<6f', packet, off)
        off += 6*4
        point_num, = struct.unpack_from('<I', packet, off)
        off += 4

        ranges = struct.unpack_from(f'<{point_num}H', packet, off)
        off += 2 * point_num
        intensities = struct.unpack_from(f'<{point_num}B', packet, off)

        angles = np.arange(point_num) * a_inc + a_min
        rs = np.array(ranges) / 1000.0
        xs = rs * np.cos(angles)
        ys = rs * np.sin(angles)

        if R_LIMIT is None:
            R_LIMIT = r_max
            ax.set_xlim(-R_LIMIT, R_LIMIT)
            ax.set_ylim(-R_LIMIT, R_LIMIT)

        xs_all.extend(xs)
        ys_all.extend(ys)
        int_all.extend(intensities)

        offsets = np.column_stack((xs_all, ys_all))
        sc.set_offsets(offsets)
        sc.set_array(np.array(int_all))

        fig.canvas.draw_idle()
        plt.pause(0.001)
