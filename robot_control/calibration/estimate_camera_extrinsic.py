#!/usr/bin/env python3

import argparse
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from tf2_ros import Buffer, TransformListener


def quaternion_to_matrix(x, y, z, w):
    n = x*x + y*y + z*z + w*w

    if n < 1e-12:
        return np.eye(3)

    s = 2.0 / n

    xx = x * x * s
    yy = y * y * s
    zz = z * z * s
    xy = x * y * s
    xz = x * z * s
    yz = y * z * s
    wx = w * x * s
    wy = w * y * s
    wz = w * z * s

    return np.array([
        [1.0 - yy - zz, xy - wz,       xz + wy],
        [xy + wz,       1.0 - xx - zz, yz - wx],
        [xz - wy,       yz + wx,       1.0 - xx - yy],
    ])


def matrix_to_rpy(R):
    # R = Rz(yaw) Ry(pitch) Rx(roll)
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)

    singular = sy < 1e-8

    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0

    return roll, pitch, yaw


def make_transform(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def average_rotations(rotations):
    M = np.zeros((3, 3))

    for R in rotations:
        M += R

    U, _, Vt = np.linalg.svd(M)
    R_avg = U @ Vt

    if np.linalg.det(R_avg) < 0:
        U[:, -1] *= -1
        R_avg = U @ Vt

    return R_avg


class ExtrinsicCollector(Node):
    def __init__(self, args):
        super().__init__("camera_extrinsic_collector")

        self.args = args

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.transforms = []
        self.last_stamp = None

        # Tag frame convention confirmed from the observed frontal tag:
        #
        # tag +x = right when looking at the printed tag
        # tag +y = upward
        # tag +z = outward from the printed face
        #
        # Base frame:
        # +x = robot forward
        # +y = robot left
        # +z = upward
        #
        # The tag is mounted vertically, facing the robot, with its top edge up.
        #
        # Therefore:
        # tag +x = -base +y
        # tag +y = +base +z
        # tag +z = -base +x

        R_base_tag = np.array([
            [ 0.0,  0.0, -1.0],
            [-1.0,  0.0,  0.0],
            [ 0.0,  1.0,  0.0],
        ])

        t_base_tag = np.array([
            args.tag_x,
            args.tag_y,
            args.tag_z,
        ])

        self.T_base_tag = make_transform(
            R_base_tag,
            t_base_tag
        )

        self.create_timer(0.02, self.collect)

        self.get_logger().info(
            f"Known tag position in base_link: "
            f"[{args.tag_x:.4f}, {args.tag_y:.4f}, {args.tag_z:.4f}] m"
        )

        self.get_logger().info(
            f"Collecting {args.samples} unique TF samples..."
        )

    def collect(self):
        if len(self.transforms) >= self.args.samples:
            self.finish()
            return

        try:
            tf = self.buffer.lookup_transform(
                self.args.camera_frame,
                self.args.tag_frame,
                Time(),
            )
        except Exception:
            return

        stamp = (
            tf.header.stamp.sec,
            tf.header.stamp.nanosec,
        )

        # Don't repeatedly use the same detector observation.
        if stamp == self.last_stamp:
            return

        self.last_stamp = stamp

        tr = tf.transform.translation
        q = tf.transform.rotation

        R_camera_tag = quaternion_to_matrix(
            q.x, q.y, q.z, q.w
        )

        t_camera_tag = np.array([
            tr.x,
            tr.y,
            tr.z,
        ])

        T_camera_tag = make_transform(
            R_camera_tag,
            t_camera_tag,
        )

        # T_base_tag = T_base_camera * T_camera_tag
        #
        # therefore:
        #
        # T_base_camera =
        # T_base_tag * inverse(T_camera_tag)

        T_base_camera = (
            self.T_base_tag
            @ np.linalg.inv(T_camera_tag)
        )

        self.transforms.append(T_base_camera)

        self.get_logger().info(
            f"sample {len(self.transforms):3d}/"
            f"{self.args.samples}"
        )

    def finish(self):
        translations = np.array(
            [T[:3, 3] for T in self.transforms]
        )

        rotations = [
            T[:3, :3] for T in self.transforms
        ]

        t_avg = translations.mean(axis=0)
        t_std = translations.std(axis=0)

        R_avg = average_rotations(rotations)

        roll, pitch, yaw = matrix_to_rpy(R_avg)

        print()
        print("=" * 60)
        print("CAMERA EXTRINSIC: base_link -> camera_frame")
        print("=" * 60)

        print()
        print("Translation [m]")
        print(f"x = {t_avg[0]:.6f}")
        print(f"y = {t_avg[1]:.6f}")
        print(f"z = {t_avg[2]:.6f}")

        print()
        print("Translation sample std [m]")
        print(f"x std = {t_std[0]:.6f}")
        print(f"y std = {t_std[1]:.6f}")
        print(f"z std = {t_std[2]:.6f}")

        print()
        print("Rotation RPY [rad]")
        print(f"roll  = {roll:.6f}")
        print(f"pitch = {pitch:.6f}")
        print(f"yaw   = {yaw:.6f}")

        print()
        print("Rotation RPY [deg]")
        print(f"roll  = {math.degrees(roll):.3f}")
        print(f"pitch = {math.degrees(pitch):.3f}")
        print(f"yaw   = {math.degrees(yaw):.3f}")

        print()
        print("Static TF command:")
        print(
            "ros2 run tf2_ros static_transform_publisher "
            f"--x {t_avg[0]:.6f} "
            f"--y {t_avg[1]:.6f} "
            f"--z {t_avg[2]:.6f} "
            f"--roll {roll:.6f} "
            f"--pitch {pitch:.6f} "
            f"--yaw {yaw:.6f} "
            "--frame-id base_link "
            "--child-frame-id camera_frame"
        )

        print("=" * 60)

        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--tag-x",
        type=float,
        required=True,
        help="Tag center forward position in base_link frame [m]",
    )

    parser.add_argument(
        "--tag-y",
        type=float,
        required=True,
        help="Tag center left position in base_link frame [m]",
    )

    parser.add_argument(
        "--tag-z",
        type=float,
        required=True,
        help="Tag center height above base_link origin [m]",
    )

    parser.add_argument(
        "--samples",
        type=int,
        default=50,
    )

    parser.add_argument(
        "--camera-frame",
        default="camera_frame",
    )

    parser.add_argument(
        "--tag-frame",
        default="tag_0",
    )

    args = parser.parse_args()

    rclpy.init()

    node = ExtrinsicCollector(args)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
