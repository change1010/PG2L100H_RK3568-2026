import struct
import zlib

import numpy as np


FRAME_WIDTH = 1920
FRAME_HEIGHT = 1080
FRAME_BYTES = 3_110_400
PACKET_COUNT = 3240
PACKET_PAYLOAD_BYTES = 960
PACKET_HEADER_BYTES = 40
PACKET_BYTES = PACKET_HEADER_BYTES + PACKET_PAYLOAD_BYTES

HEADER = struct.Struct(">4sBBHIIHHIHHHHII")


def pack_rgb444(rgb):
    """将 1920×1080 RGB888 图像打包为逐像素 RGB444 字节流。"""
    expected_shape = (FRAME_HEIGHT, FRAME_WIDTH, 3)
    if rgb.shape != expected_shape or rgb.dtype != np.uint8:
        raise ValueError("rgb must be uint8 shape (1080,1920,3)")

    pixels = rgb.reshape(-1, 2, 3) >> 4
    packed = np.empty((pixels.shape[0], 3), dtype=np.uint8)
    packed[:, 0] = (pixels[:, 0, 0] << 4) | pixels[:, 0, 1]
    packed[:, 1] = (pixels[:, 0, 2] << 4) | pixels[:, 1, 0]
    packed[:, 2] = (pixels[:, 1, 1] << 4) | pixels[:, 1, 2]
    return packed.tobytes()


def iter_packets(packed, session, frame):
    """按传输协议把一帧字节流拆分为带校验头的数据包。"""
    if len(packed) != FRAME_BYTES:
        raise ValueError

    session_is_valid = isinstance(session, int) and 0 <= session <= 0xFFFFFFFF
    frame_is_valid = isinstance(frame, int) and 0 <= frame <= 0xFFFFFFFF
    if not session_is_valid or not frame_is_valid:
        raise ValueError

    for packet_index in range(PACKET_COUNT):
        offset = packet_index * PACKET_PAYLOAD_BYTES
        payload = packed[offset : offset + PACKET_PAYLOAD_BYTES]
        header = HEADER.pack(
            b"QSV1",
            1,
            1,
            PACKET_HEADER_BYTES,
            session,
            frame,
            packet_index,
            PACKET_COUNT,
            offset,
            PACKET_PAYLOAD_BYTES,
            FRAME_WIDTH,
            FRAME_HEIGHT,
            0,
            FRAME_BYTES,
            0,
        )
        checksum = zlib.crc32(header[:36] + payload) & 0xFFFFFFFF
        yield header[:36] + struct.pack(">I", checksum) + payload


def parse_datagram(data):
    """校验并解析单个完整数据包，返回其会话、帧号、序号和负载。"""
    if len(data) != PACKET_BYTES:
        raise ValueError

    (
        magic,
        version,
        pixel_format,
        header_size,
        session,
        frame,
        packet_index,
        packet_count,
        offset,
        payload_size,
        width,
        height,
        reserved,
        frame_size,
        checksum,
    ) = HEADER.unpack(data[:PACKET_HEADER_BYTES])

    expected_values = (
        magic == b"QSV1",
        version == 1,
        pixel_format == 1,
        header_size == PACKET_HEADER_BYTES,
        packet_count == PACKET_COUNT,
        packet_index < PACKET_COUNT,
        offset == packet_index * PACKET_PAYLOAD_BYTES,
        payload_size == PACKET_PAYLOAD_BYTES,
        width == FRAME_WIDTH,
        height == FRAME_HEIGHT,
        reserved == 0,
        frame_size == FRAME_BYTES,
    )
    if not all(expected_values):
        raise ValueError

    payload = data[PACKET_HEADER_BYTES:]
    actual_checksum = zlib.crc32(data[:36] + payload) & 0xFFFFFFFF
    if actual_checksum != checksum:
        raise ValueError

    return {
        "session": session,
        "frame": frame,
        "index": packet_index,
        "payload": payload,
    }


def decode_downsample_rgb565(packed):
    """解码 RGB444 字节流，并按每两行、每两列抽样为 RGB565。"""
    if len(packed) != FRAME_BYTES:
        raise ValueError("packed length must be 3110400")

    byte_values = np.frombuffer(packed, np.uint8).reshape(-1, 3).astype(np.uint16)
    first_pixel = (
        ((byte_values[:, 0] >> 4) << 12)
        | ((byte_values[:, 0] & 15) << 7)
        | ((byte_values[:, 1] >> 4) << 1)
    )
    second_pixel = (
        ((byte_values[:, 1] & 15) << 12)
        | ((byte_values[:, 2] >> 4) << 7)
        | ((byte_values[:, 2] & 15) << 1)
    )

    pixels = np.empty(byte_values.shape[0] * 2, dtype=np.uint16)
    pixels[::2] = first_pixel
    pixels[1::2] = second_pixel
    image = pixels.reshape(FRAME_HEIGHT, FRAME_WIDTH)
    return image[::2, ::2].copy()
