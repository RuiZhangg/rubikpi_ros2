#!/usr/bin/env python3

import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml


def generate_launch_description():
    run_controller = LaunchConfiguration("run_controller")
    run_slam = LaunchConfiguration("run_slam")
    shape = LaunchConfiguration("shape")
    side_length_m = LaunchConfiguration("side_length_m")
    laps = LaunchConfiguration("laps")
    run_name = LaunchConfiguration("run_name")

    hw3_config = PathJoinSubstitution(
        [FindPackageShare("robot_control"), "config", "hw3.yaml"]
    )
    tag_config = PathJoinSubstitution(
        [FindPackageShare("apriltag_ros"), "cfg", "tags_36h11.yaml"]
    )
    camera_config = PathJoinSubstitution(
        [
            FindPackageShare("robot_vision_camera"),
            "config",
            "camera_parameter.yaml",
        ]
    )
    extrinsic_path = os.path.join(
        get_package_share_directory("robot_control"),
        "config",
        "camera_extrinsics.yaml",
    )
    with open(extrinsic_path, "r", encoding="utf-8") as stream:
        extrinsic = yaml.safe_load(stream)["camera_extrinsics"]
    translation = extrinsic["translation"]
    rotation = extrinsic["rotation_rpy"]

    declarations = [
        DeclareLaunchArgument("run_controller", default_value="false"),
        DeclareLaunchArgument("run_slam", default_value="true"),
        DeclareLaunchArgument("shape", default_value="square"),
        DeclareLaunchArgument("side_length_m", default_value="1.8"),
        DeclareLaunchArgument("laps", default_value="1"),
        DeclareLaunchArgument(
            "run_name", default_value=time.strftime("run_%Y%m%d_%H%M%S")
        ),
    ]

    camera = Node(
        package="robot_vision_camera",
        executable="robot_vision_camera_node",
        name="robot_vision_camera",
        output="screen",
        parameters=[
            {
                "frame_rate": 10,
                "width": 1280,
                "height": 720,
                "input_format": "NV12",
                "output_format": "BGR",
                "topic_name": "camera/image_raw",
                "image_compress": True,
                "image_rectify": False,
                "camera_parameter_path": camera_config,
                "jpeg_quality": 90,
                "max_threads": 1,
            }
        ],
    )

    rectify = Node(
        package="image_proc",
        executable="rectify_node",
        name="rectify_node",
        output="screen",
        remappings=[
            ("image", "/camera/image_raw"),
            ("camera_info", "/camera/camera_info"),
            ("image_rect", "/camera/image_rect"),
        ],
    )

    detector = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag_node",
        output="screen",
        parameters=[tag_config],
        remappings=[
            ("image_rect", "/camera/image_rect"),
            ("camera_info", "/camera/camera_info"),
        ],
    )

    camera_extrinsic = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_extrinsic_publisher",
        arguments=[
            "--x",
            str(translation["x"]),
            "--y",
            str(translation["y"]),
            "--z",
            str(translation["z"]),
            "--roll",
            str(rotation["roll"]),
            "--pitch",
            str(rotation["pitch"]),
            "--yaw",
            str(rotation["yaw"]),
            "--frame-id",
            str(extrinsic["parent_frame"]),
            "--child-frame-id",
            str(extrinsic["child_frame"]),
        ],
        output="screen",
    )

    slam = Node(
        package="robot_control",
        executable="hw3_slam",
        name="hw3_slam_node",
        output="screen",
        condition=IfCondition(run_slam),
        parameters=[hw3_config, {"run_name": run_name}],
    )

    motor = Node(
        package="robot_control",
        executable="motor_control",
        name="motor_controller_node",
        output="screen",
    )
    velocity_mapping = Node(
        package="robot_control",
        executable="velocity_mapping",
        name="velocity_to_motor_node",
        output="screen",
    )

    controller = Node(
        package="robot_control",
        executable="hw3_controller",
        name="hw3_controller",
        output="screen",
        condition=IfCondition(run_controller),
        parameters=[
            hw3_config,
            {
                "shape": shape,
                "side_length_m": ParameterValue(side_length_m, value_type=float),
                "laps": ParameterValue(laps, value_type=int),
                "run_name": run_name,
            },
        ],
    )

    delayed_controller = TimerAction(period=5.0, actions=[controller])
    return LaunchDescription(
        declarations
        + [
            camera,
            rectify,
            detector,
            camera_extrinsic,
            slam,
            motor,
            velocity_mapping,
            delayed_controller,
        ]
    )
