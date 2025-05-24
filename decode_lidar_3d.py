import struct
import binascii

MAGIC = b"\x55\xAA\x05\x0A"
EXPECTED_TYPE = 102  # 3D point packet type, no IMU

def decode_packet(packet: bytes) -> dict:
    # Minimal length: UDP header (8) + FrameHeader (12) + FrameTail (12)
    if len(packet) < 8 + 12 + 12:
        raise ValueError(f"Packet too short, got {len(packet)} bytes")

    # Parse UDP header (big-endian)
    src_port, dst_port, udp_length, udp_checksum = struct.unpack("!HHHH", packet[:8])

    # Strip off UDP header
    payload = packet[8:]

    # Parse FrameHeader (little-endian)
    magic, packet_type, packet_size = struct.unpack_from("<4sII", payload, 0)
    if magic != MAGIC:
        raise ValueError(f"Bad magic: {magic!r}")
    if packet_type != EXPECTED_TYPE:
        raise ValueError(f"Unexpected packet_type: {packet_type}")

    # Ensure payload length matches the header’s packet_size
    if len(payload) != packet_size:
        raise ValueError(f"Payload size mismatch: header says {packet_size}, actual {len(payload)}")

    # DataInfo (16 B at offset 12)
    seq, payload_size, sec, nsec = struct.unpack_from("<IIII", payload, 12)
    # Sanity check: payload_size == packet_size - header(12) - tail(12)
    if payload_size != packet_size - 24:
        raise ValueError(f"payload_size mismatch: header says {payload_size}, expected {packet_size - 24}")

    # InsideState (36 B at offset 28)
    (
        sys_rotation_period,
        com_rotation_period,
        dirty_index,
        packet_lost_up,
        packet_lost_down,
        apd_temperature,
        apd_voltage,
        laser_voltage,
        imu_temperature
    ) = struct.unpack_from("<IIfffffff", payload, 28)

    # Calibration params (32 B at offset 64)
    (
        a_axis_dist,
        b_axis_dist,
        theta_angle_bias,
        alpha_angle_bias,
        beta_angle,
        xi_angle,
        range_bias,
        range_scale
    ) = struct.unpack_from("<ffffffff", payload, 64)

    # Scan geometry and timing (32 B at offset 96)
    (
        h_angle_start,
        h_angle_step,
        scan_period,
        range_min,
        range_max,
        angle_min,
        angle_increment,
        time_increment
    ) = struct.unpack_from("<ffffffff", payload, 96)

    # Point data begins at offset 128
    point_num, = struct.unpack_from("<I", payload, 128)
    if point_num > 300:
        raise ValueError(f"point_num {point_num} exceeds max 300")

    # Fixed-size arrays of 300
    ranges      = struct.unpack_from("<300H", payload, 132)
    intensities = struct.unpack_from("<300B", payload, 732)

    points = [
        {"distance_mm": ranges[i], "intensity": intensities[i]}
        for i in range(point_num)
    ]

    # FrameTail is the last 12 bytes of payload
    tail_off = packet_size - 12
    crc32_recv, msg_type_check = struct.unpack_from("<II", payload, tail_off)
    reserve, tail_bytes = struct.unpack_from("<2s2s", payload, tail_off + 8)

    # Compute and compare CRC32
    crc_computed = binascii.crc32(payload[:tail_off]) & 0xFFFFFFFF

    return {
        "udp_header": {
            "src_port": src_port,
            "dst_port": dst_port,
            "length": udp_length,
            "checksum": udp_checksum,
        },
        "lidar_header": {
            "magic": magic,
            "packet_type": packet_type,
            "packet_size": packet_size,
        },
        "data_info": {
            "seq": seq,
            "payload_size": payload_size,
            "sec": sec,
            "nsec": nsec,
        },
        "inside_state": {
            "sys_rotation_period": sys_rotation_period,
            "com_rotation_period": com_rotation_period,
            "dirty_index": dirty_index,
            "packet_lost_up": packet_lost_up,
            "packet_lost_down": packet_lost_down,
            "apd_temperature": apd_temperature,
            "apd_voltage": apd_voltage,
            "laser_voltage": laser_voltage,
            "imu_temperature": imu_temperature,
        },
        "calib_param": {
            "a_axis_dist": a_axis_dist,
            "b_axis_dist": b_axis_dist,
            "theta_angle_bias": theta_angle_bias,
            "alpha_angle_bias": alpha_angle_bias,
            "beta_angle": beta_angle,
            "xi_angle": xi_angle,
            "range_bias": range_bias,
            "range_scale": range_scale,
        },
        "geometry": {
            "h_angle_start": h_angle_start,
            "h_angle_step": h_angle_step,
            "scan_period": scan_period,
            "range_min": range_min,
            "range_max": range_max,
            "angle_min": angle_min,
            "angle_increment": angle_increment,
            "time_increment": time_increment,
        },
        "point_num": point_num,
        "points": points,
        "frame_tail": {
            "crc32_recv": crc32_recv,
            "msg_type_check": msg_type_check,
            "reserve": reserve,
            "tail_bytes": tail_bytes,
        },
        "crc_check": {
            "received": crc32_recv,
            "computed": crc_computed,
            "ok": (crc32_recv == crc_computed),
        },
    }
