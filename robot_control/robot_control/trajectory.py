"""Pure geometry helpers for closed square and regular-octagon paths."""

import math
from typing import List

import numpy as np


def build_waypoints(shape: str, side_length_m: float, laps: int) -> List[np.ndarray]:
    if side_length_m <= 0.0:
        raise ValueError("side_length_m must be positive")
    if laps < 1:
        raise ValueError("laps must be at least one")

    normalized_shape = shape.strip().lower()
    if normalized_shape == "square":
        edge_count = 4
        heading_step = math.pi / 2.0
    elif normalized_shape == "octagon":
        edge_count = 8
        heading_step = math.pi / 4.0
    else:
        raise ValueError("shape must be 'square' or 'octagon'")

    waypoints = [np.zeros(2, dtype=float)]
    current = waypoints[0].copy()
    for _ in range(laps):
        for edge_index in range(edge_count):
            heading = edge_index * heading_step
            current = current + side_length_m * np.array(
                [math.cos(heading), math.sin(heading)], dtype=float
            )
            waypoints.append(current.copy())

    if np.linalg.norm(waypoints[-1]) < 1e-10:
        waypoints[-1][:] = 0.0
    return waypoints
