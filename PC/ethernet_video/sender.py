import argparse
import ctypes
import os
import secrets
import socket
import sys
import time

import numpy as np
from PIL import Image, ImageGrab

sys.path.insert(
    0,
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
)

from PC.ethernet_video.pattern_source import make_diagnostic_pattern
from PC.ethernet_video.protocol import iter_packets, pack_rgb444, parse_datagram


def enable_dpi_awareness():
    """启用 Windows 高 DPI 感知；不支持时保留系统默认设置。"""
    try:
        ctypes.windll.shcore.SetProcessDpiAwareness(2)
    except Exception:
        pass


def capture_desktop():
    """捕获主显示器并按比例缩放到协议规定的画布尺寸。"""
    image = ImageGrab.grab(all_screens=False).convert("RGB")
    source_width, source_height = image.size
    scale = min(1920 / source_width, 1080 / source_height)
    output_width = max(1, int(source_width * scale))
    output_height = max(1, int(source_height * scale))
    image = image.resize((output_width, output_height), Image.BILINEAR)

    canvas = Image.new("RGB", (1920, 1080), (0, 0, 0))
    left = (1920 - output_width) // 2
    top = (1080 - output_height) // 2
    canvas.paste(image, (left, top))
    return np.asarray(canvas, dtype=np.uint8)


def make_synthetic_frame(frame_number):
    """生成带移动色条的合成帧，用于发送链路检查。"""
    image = np.zeros((1080, 1920, 3), dtype=np.uint8)
    image[:540, :, 0] = 255
    image[540:, :, 1] = 255

    bar_left = (frame_number * 20) % 1920
    bar_right = min(bar_left + 40, 1920)
    image[:, bar_left:bar_right, :] = 220
    return image


def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--target", default="192.168.1.100")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--bind")
    parser.add_argument(
        "--source",
        choices=("desktop", "synthetic", "pattern"),
        default="desktop",
    )
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--frames", type=int, default=0)
    parser.add_argument("--fps", type=float, default=30)
    return parser.parse_args()


def create_source_frame(source, frame_number, diagnostic_pattern):
    if source == "desktop":
        return capture_desktop()
    if source == "pattern":
        return diagnostic_pattern
    return make_synthetic_frame(frame_number)


def main():
    args = parse_arguments()
    fps = min(args.fps, 30)
    frame_interval = 1.0 / fps if fps > 0 else 0.0

    if args.dry_run:
        source = "synthetic" if args.source == "desktop" else args.source
        frame_limit = 2 if args.frames == 0 else args.frames
    else:
        source = args.source
        frame_limit = args.frames

    if source == "desktop":
        enable_dpi_awareness()

    diagnostic_pattern = None
    if source == "pattern":
        diagnostic_pattern = np.asarray(make_diagnostic_pattern(), dtype=np.uint8)

    udp_socket = None
    if not args.dry_run:
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4 * 1024 * 1024)
        if args.bind:
            udp_socket.bind((args.bind, 0))
        udp_socket.connect((args.target, args.port))

    session_id = secrets.randbits(32)
    frame_number = 0
    sent_frames = 0
    sent_packets = 0
    dropped_frames = 0
    start_time = time.monotonic()
    next_frame_time = start_time

    try:
        while frame_limit == 0 or frame_number < frame_limit:
            if frame_interval:
                now = time.monotonic()
                if next_frame_time > now:
                    time.sleep(next_frame_time - now)

                next_frame_time += frame_interval
                if time.monotonic() > next_frame_time + frame_interval:
                    next_frame_time = time.monotonic() + frame_interval

            rgb = create_source_frame(source, frame_number, diagnostic_pattern)
            packed_frame = pack_rgb444(rgb)
            assert len(packed_frame) == 3_110_400

            packets = list(
                iter_packets(packed_frame, session_id, frame_number & 0xFFFFFFFF)
            )
            assert len(packets) == 3240

            if args.dry_run:
                for packet_index, packet in enumerate(packets):
                    assert len(packet) == 1000
                    packet_info = parse_datagram(packet)
                    assert packet_info["session"] == session_id
                    assert packet_info["frame"] == (frame_number & 0xFFFFFFFF)
                    assert packet_info["index"] == packet_index

                sent_frames += 1
                sent_packets += len(packets)
            else:
                frame_sent = True
                try:
                    for packet in packets:
                        assert len(packet) == 1000
                        bytes_sent = udp_socket.send(packet)
                        if bytes_sent != 1000:
                            raise OSError("short send")
                        sent_packets += 1
                except OSError as error:
                    dropped_frames += 1
                    frame_sent = False
                    sys.stderr.write(f"drop frame {frame_number}: {error}\n")

                if frame_sent:
                    sent_frames += 1

            frame_number += 1
    except KeyboardInterrupt:
        pass
    finally:
        if udp_socket is not None:
            udp_socket.close()

    elapsed = time.monotonic() - start_time
    actual_fps = frame_number / elapsed if elapsed > 0 else 0.0
    print(
        "frames=%d packets=%d dropped=%d elapsed=%.3f actual_fps=%.2f"
        % (frame_number, sent_packets, dropped_frames, elapsed, actual_fps)
    )


if __name__ == "__main__":
    main()
