#!/usr/bin/env python3
"""Generate the default 0.05 m navigation map from Gazebo collision geometry."""

from __future__ import annotations

import argparse
from pathlib import Path


RESOLUTION = 0.05
ORIGIN_X = -8.0
ORIGIN_Y = -6.0
WIDTH_METERS = 16.0
HEIGHT_METERS = 12.0

# Axis-aligned collision rectangles copied from pipeline_world.sdf. Keeping
# this vector description next to the generator avoids pretending that a
# nearest-neighbour enlargement of the old 0.5 m image adds map information.
RECTANGLES = (
    (-8.0, 8.0, 5.5, 6.0),       # north wall
    (-8.0, 8.0, -6.0, -5.5),     # south wall
    (7.5, 8.0, -6.0, 6.0),       # east wall
    (-8.0, -7.5, -6.0, 6.0),     # west wall
    (-3.25, -2.25, -0.5, 3.5),   # pipe rack A
    (1.75, 2.75, -4.0, 0.0),     # pipe rack B
    (-0.5, 1.0, 0.5, 1.5),       # control cabinet
)


def generate_pgm() -> bytes:
    width = round(WIDTH_METERS / RESOLUTION)
    height = round(HEIGHT_METERS / RESOLUTION)
    pixels = bytearray([254] * (width * height))
    for image_row in range(height):
        map_row = height - 1 - image_row
        y = ORIGIN_Y + (map_row + 0.5) * RESOLUTION
        for column in range(width):
            x = ORIGIN_X + (column + 0.5) * RESOLUTION
            if any(
                min_x <= x <= max_x and min_y <= y <= max_y
                for min_x, max_x, min_y, max_y in RECTANGLES
            ):
                pixels[image_row * width + column] = 0
    header = (
        'P5\n'
        '# 16 m x 12 m demo map generated from pipeline_world.sdf collisions.\n'
        f'{width} {height}\n255\n'
    ).encode('ascii')
    return header + pixels


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    output = root / 'src/patrol_robot_navigation/maps/pipeline_map.pgm'
    generated = generate_pgm()
    if args.check:
        if not output.is_file() or output.read_bytes() != generated:
            raise SystemExit('pipeline_map.pgm 与生成器不一致')
        print('pipeline_map.pgm 校验通过')
        return
    output.write_bytes(generated)
    print(f'已生成 {output}：320 × 240，{RESOLUTION:.2f} m/格')


if __name__ == '__main__':
    main()
