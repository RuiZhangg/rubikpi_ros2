#!/usr/bin/env python3

"""List or reject unresolved HW3 report placeholders before submission."""

import argparse
import re


REQUIRED_PHRASES = [
    "State vector",
    "Control and motion model",
    "System matrix",
    "Measurement vector",
    "Measurement matrix",
    "Initial state and covariance",
    "Process noise",
    "Measurement noise",
    "First observation of a new landmark",
    "Landmark outside the field of view",
    "Previously observed landmark reappearing",
    "Square - one lap",
    "Square - multiple laps",
    "Octagon - one lap",
    "Octagon - multiple laps",
    "World-frame transform and landmark error",
    "Square versus octagon maps",
    "Does the map improve over multiple laps?",
    "Video and code submission",
]


def check(path):
    with open(path, "r", encoding="utf-8") as stream:
        text = stream.read()
    missing_sections = [phrase for phrase in REQUIRED_PHRASES if phrase not in text]
    placeholders = []
    for line_number, line in enumerate(text.splitlines(), start=1):
        for match in re.finditer(r"TODO\[[A-Z/]+\]", line):
            placeholders.append((line_number, match.group(0), line.strip()))
    return missing_sections, placeholders


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("report", nargs="?", default="report/hw3_report.md")
    parser.add_argument(
        "--final",
        action="store_true",
        help="exit nonzero if any placeholder remains",
    )
    args = parser.parse_args()
    missing_sections, placeholders = check(args.report)

    for phrase in missing_sections:
        print(f"MISSING SECTION: {phrase}")
    for line_number, token, line in placeholders:
        print(f"{args.report}:{line_number}: {token}: {line}")
    print(
        f"summary: {len(missing_sections)} missing required sections, "
        f"{len(placeholders)} unresolved placeholders"
    )
    if missing_sections or (args.final and placeholders):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
