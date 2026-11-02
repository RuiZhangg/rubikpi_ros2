#!/usr/bin/env python3

import argparse
import math
import os
import yaml
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from tf2_ros import Buffer, TransformListener


def quaternion_to_matrix(x, y, z, w):
    n = x*x + y*y + z*z + w*w
    if n < 1e-12:
        return np.eye(3)

    s = 2.0 / n
    xx, yy, zz = x*x*s, y*y*s, z*z*s
    xy, xz, yz = x*y*s, x*z*s, y*z*s
    wx, wy, wz = w*x*s, w*y*s, w*z*s

    return np.array([
        [1.0 - yy - zz, xy - wz,       xz + wy],
        [xy + wz,       1.0 - xx - zz, yz - wx],
        [xz - wy,       yz + wx,       1.0 - xx - yy],
    ])


def make_transform(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t, dtype=float)
    return T


def transform_msg_to_matrix(tf_msg):
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    R = quaternion_to_matrix(q.x, q.y, q.z, q.w)
    return make_transform(R, [t.x, t.y, t.z])


def wrap_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def world_tag_transform(x, y, z, face_yaw):
    """
    AprilTag frame convention inferred/validated from the detector:
      +x: to the right on the printed tag
      +y: upward on the printed tag
      +z: outward normal from the printed face

    For an upright vertical tag:
      tag +y = world +z
      tag +z = horizontal face normal
      tag +x completes a right-handed frame.
    """
    n = np.array([
        math.cos(face_yaw),
        math.sin(face_yaw),
        0.0,
    ])  # tag +z

    y_tag = np.array([0.0, 0.0, 1.0])
    x_tag = np.cross(y_tag, n)

    R = np.column_stack((x_tag, y_tag, n))
    return make_transform(R, [x, y, z])


class LandmarkLocalizer(Node):
    def __init__(self, map_file, max_age):
        super().__init__("landmark_localizer")

        with open(map_file, "r") as f:
            data = yaml.safe_load(f)

        self.tags = {}
        for tag_id_raw, entry in data["tags"].items():
            tag_id = int(tag_id_raw)
            self.tags[tag_id] = world_tag_transform(
                float(entry["x"]),
                float(entry["y"]),
                float(entry["z"]),
                float(entry["face_yaw"]),
            )

        self.max_age = max_age

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_pub = self.create_publisher(
            PoseStamped,
            "/hw2/vision_pose",
            10,
        )
        self.tag_pub = self.create_publisher(
            Int32,
            "/hw2/vision_tag_id",
            10,
        )

        self.last_report = 0.0
        self.create_timer(0.10, self.update)

        self.get_logger().info(
            f"Loaded {len(self.tags)} landmarks from {map_file}: "
            f"{sorted(self.tags.keys())}"
        )
        self.get_logger().info(
            "Publishing planar world pose on /hw2/vision_pose"
        )

    def lookup_candidate(self, tag_id):
        tag_frame = f"tag_{tag_id}"

        try:
            # base_link -> tag_i is available through:
            # base_link -> camera_frame (static extrinsic)
            # camera_frame -> tag_i (AprilTag detection)
            tf_msg = self.tf_buffer.lookup_transform(
                "base_link",
                tag_frame,
                Time(),
            )
        except Exception:
            return None

        stamp = rclpy.time.Time.from_msg(tf_msg.header.stamp)
        now = self.get_clock().now()
        age = (now - stamp).nanoseconds / 1e9

        if age < 0.0 or age > self.max_age:
            return None

        T_base_tag = transform_msg_to_matrix(tf_msg)

        # Known relation:
        #   T_world_tag = T_world_base * T_base_tag
        # Therefore:
        #   T_world_base = T_world_tag * inverse(T_base_tag)
        T_world_base = (
            self.tags[tag_id]
            @ np.linalg.inv(T_base_tag)
        )

        # Prefer closer tags because PnP translation/orientation generally
        # becomes less stable as the tag occupies fewer pixels.
        distance = np.linalg.norm(T_base_tag[:3, 3])

        return {
            "tag_id": tag_id,
            "T_world_base": T_world_base,
            "distance": float(distance),
            "age": float(age),
        }

    def update(self):
        candidates = []

        for tag_id in self.tags:
            result = self.lookup_candidate(tag_id)
            if result is not None:
                candidates.append(result)

        if not candidates:
            return

        # Minimal robust policy for HW2:
        # use the closest currently visible mapped tag.
        best = min(candidates, key=lambda c: c["distance"])

        T = best["T_world_base"]
        R = T[:3, :3]

        x = float(T[0, 3])
        y = float(T[1, 3])
        yaw = wrap_angle(math.atan2(R[1, 0], R[0, 0]))

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self.pose_pub.publish(msg)

        id_msg = Int32()
        id_msg.data = best["tag_id"]
        self.tag_pub.publish(id_msg)

        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self.last_report > 1.0:
            self.last_report = now_sec
            self.get_logger().info(
                f"tag={best['tag_id']} "
                f"range={best['distance']:.3f} m "
                f"pose=({x:.3f}, {y:.3f}, "
                f"{math.degrees(yaw):.1f} deg)"
            )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--map",
        default=os.path.expanduser("~/landmarks.yaml"),
        help="landmark YAML file",
    )
    parser.add_argument(
        "--max-age",
        type=float,
        default=1.0,
        help="maximum accepted AprilTag TF age in seconds",
    )

    args = parser.parse_args()

    rclpy.init()
    node = LandmarkLocalizer(args.map, args.max_age)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
