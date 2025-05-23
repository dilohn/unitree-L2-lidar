import struct
import binascii
import numpy as np


def decode_packet(packet: bytes) -> dict:
    if len(packet) != 1044:
        raise ValueError(f"Expected 1044 bytes, got {len(packet)}")

    if packet[0:4] != b"\x55\xaa\x05\x0a":
        raise ValueError("Bad magic header")
    packet_type, packet_size = struct.unpack_from("<II", packet, 4)

    off = 12
    seq, payload_size, stamp_sec, stamp_nsec = struct.unpack_from("<IIII", packet, off)
    off += 16

    sys_rot_period, com_rot_period = struct.unpack_from("<II", packet, off)
    temps_voltages = struct.unpack_from("<7f", packet, off + 8)
    off += 36

    calib_keys = (
        "a_axis_dist",
        "b_axis_dist",
        "theta_bias",
        "alpha_bias",
        "beta_angle",
        "xi_angle",
        "range_bias",
        "range_scale",
    )
    calib_vals = struct.unpack_from("<8f", packet, off)
    calibration = dict(zip(calib_keys, calib_vals))
    off += 32

    scan_keys = (
        "horiz_start",
        "horiz_step",
        "scan_period",
        "range_min",
        "range_max",
        "angle_min",
        "angle_inc",
        "time_inc",
    )
    scan_vals = struct.unpack_from("<8f", packet, off)
    scan_info = dict(zip(scan_keys, scan_vals))
    off += 32

    (point_num,) = struct.unpack_from("<I", packet, off)
    off += 4

    ranges_raw = np.frombuffer(packet, dtype="<H", count=point_num, offset=off)
    off += point_num * 2
    intensities = np.frombuffer(packet, dtype="B", count=point_num, offset=off)
    off += point_num

    tail_crc, msg_type_check = struct.unpack_from("<II", packet, off)
    crc_valid = (binascii.crc32(packet[:1032]) & 0xFFFFFFFF) == tail_crc

    rbias = calibration["range_bias"]
    rscale = calibration["range_scale"]
    ranges_m = (ranges_raw.astype(np.float32) * rscale + rbias) * 1e-3

    th = (
        scan_info["horiz_start"]
        + np.arange(point_num) * scan_info["horiz_step"]
        + calibration["theta_bias"]
    )
    ph = (
        scan_info["angle_min"]
        + np.arange(point_num) * scan_info["angle_inc"]
        + calibration["alpha_bias"]
    )

    x = ranges_m * np.cos(ph) * np.cos(th)
    y = ranges_m * np.cos(ph) * np.sin(th)
    z = ranges_m * np.sin(ph)

    points = np.stack((x, y, z, intensities.astype(np.float32)), axis=1)

    return {
        "packet_type": packet_type,
        "seq": seq,
        "timestamp": (stamp_sec, stamp_nsec),
        "sys_rot_period": sys_rot_period,
        "com_rot_period": com_rot_period,
        "temps_voltages": temps_voltages,
        "calibration": calibration,
        "scan_info": scan_info,
        "point_num": point_num,
        "points": points,
        "crc_valid": crc_valid,
    }


def decode_file(path: str) -> dict:
    with open(path, "rb") as f:
        data = f.read(1044)
    return decode_packet(data)


def _print_summary(res: dict):
    print(f"Packet Type: {res['packet_type']}  Seq: {res['seq']}")
    sec, nsec = res["timestamp"]
    print(f"Timestamp: {sec}s {nsec}ns  CRC OK: {res['crc_valid']}")
    print("Calibration:", res["calibration"])
    print("Scan info:", res["scan_info"])
    print(f"Points ({res['point_num']}):")
    for i, (x, y, z, inten) in enumerate(res["points"]):
        print(f"{i:03d}: x={x:.3f} y={y:.3f} z={z:.3f}  inten={int(inten)}")


if __name__ == "__main__":
    import sys

    if len(sys.argv) != 2:
        print("Usage: python decode_lidar.py raw_packet.bin")
        sys.exit(1)
    result = decode_file(sys.argv[1])
    _print_summary(result)
