#!/usr/bin/env python3

"""Print centered square and octagon floor-mark coordinates."""

import argparse
import math


def square_markers(width, height, side):
    x_low = 0.5 * (width - side)
    y_low = 0.5 * (height - side)
    x_high = x_low + side
    y_high = y_low + side
    return [
        ("S0", x_low, y_low, "east/start"),
        ("S1", x_high, y_low, "north"),
        ("S2", x_high, y_high, "west"),
        ("S3", x_low, y_high, "south"),
    ]


def octagon_markers(width, height, side):
    span = side * (1.0 + math.sqrt(2.0))
    margin_x = 0.5 * (width - span)
    margin_y = 0.5 * (height - span)
    x = margin_x + side / math.sqrt(2.0)
    y = margin_y
    headings = [
        "east/start",
        "northeast",
        "north",
        "northwest",
        "west",
        "southwest",
        "south",
        "southeast",
    ]
    markers = []
    for index in range(8):
        markers.append((f"O{index}", x, y, headings[index]))
        heading = index * math.pi / 4.0
        x += side * math.cos(heading)
        y += side * math.sin(heading)
    return markers


def print_table(title, markers):
    print(title)
    print("mark,x_m,y_m,departing_direction")
    for name, x, y, direction in markers:
        print(f"{name},{x:.4f},{y:.4f},{direction}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--width", type=float, default=3.302)
    parser.add_argument("--height", type=float, default=3.302)
    parser.add_argument("--square-side", type=float, default=1.8)
    parser.add_argument("--octagon-side", type=float, default=0.75)
    args = parser.parse_args()

    if min(args.width, args.height, args.square_side, args.octagon_side) <= 0.0:
        raise ValueError("all dimensions must be positive")
    octagon_span = args.octagon_side * (1.0 + math.sqrt(2.0))
    if args.square_side >= min(args.width, args.height):
        raise ValueError("square path does not fit inside the arena")
    if octagon_span >= min(args.width, args.height):
        raise ValueError("octagon path does not fit inside the arena")

    print_table(
        "CENTERED SQUARE",
        square_markers(args.width, args.height, args.square_side),
    )
    print()
    print_table(
        "CENTERED OCTAGON",
        octagon_markers(args.width, args.height, args.octagon_side),
    )


if __name__ == "__main__":
    main()
