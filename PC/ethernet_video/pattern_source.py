import argparse
from pathlib import Path

from PIL import Image, ImageDraw
import numpy as np


def make_diagnostic_pattern():
    """生成用于检查像素、行边界和图像位置的静态图。"""
    y_coordinates, x_coordinates = np.indices((1080, 1920))
    rgb = np.empty((1080, 1920, 3), dtype=np.uint8)
    rgb[:, :, 0] = ((x_coordinates // 32) % 16) * 16
    rgb[:, :, 1] = ((y_coordinates // 16) % 16) * 16
    rgb[:, :, 2] = (((x_coordinates // 64) + (y_coordinates // 32)) % 16) * 16

    checkerboard = ((x_coordinates // 2 + y_coordinates // 2) & 1) * 240
    rgb[600:, :, :] = checkerboard[600:, :, None]

    image = Image.fromarray(rgb)
    drawing = ImageDraw.Draw(image)
    for column in range(0, 1920, 32):
        drawing.line((column, 0, column, 599), fill=(255, 255, 255))

    for column in range(0, 1920, 512):
        drawing.line((column, 0, column, 1079), fill=(255, 0, 255), width=2)
        drawing.rectangle((column + 4, 8, column + 210, 38), fill=(0, 0, 0))
        drawing.text(
            (column + 8, 16),
            f"512px GUIDE x={column}",
            fill=(255, 255, 255),
        )

    for row in range(64, 600, 64):
        drawing.line((0, row, 1919, row), fill=(255, 255, 255))
        drawing.text((8, row + 6), f"row={row}", fill=(255, 255, 255))

    drawing.rectangle((680, 240, 1240, 400), fill=(0, 0, 0))
    guide_lines = (
        "STATIC ALIGNMENT / RGB444 -> RGB565",
        "32 output pixels = 1 AXI beat",
        "512 output pixels = 16 AXI beats",
        "CMOS1 -> CMOS2 -> HDMI -> ETH -> QUAD",
    )
    for line_index, text in enumerate(guide_lines):
        drawing.text((700, 260 + 32 * line_index), text, fill=(255, 255, 255))

    return image


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    make_diagnostic_pattern().save(args.output)
    print(args.output.resolve())


if __name__ == "__main__":
    main()
