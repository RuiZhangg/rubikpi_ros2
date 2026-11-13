#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    kind = LaunchConfiguration("kind")
    trial_id = LaunchConfiguration("trial_id")
    minimum_samples = LaunchConfiguration("minimum_samples_per_tag")
    wait_s = LaunchConfiguration("max_observation_wait_s")

    support = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("robot_control"),
                    "launch",
                    "hw3_bringup.launch.py",
                ]
            )
        ),
        launch_arguments={
            "run_slam": "false",
            "run_controller": "false",
            "run_name": "motion_calibration_support",
        }.items(),
    )

    calibration = Node(
        package="robot_control",
        executable="motion_calibration",
        name="motion_calibration_node",
        output="screen",
        arguments=[
            kind,
            "--trial-id",
            trial_id,
            "--minimum-samples-per-tag",
            minimum_samples,
            "--max-observation-wait-s",
            wait_s,
        ],
    )
    delayed_calibration = TimerAction(period=5.0, actions=[calibration])
    stop_support = RegisterEventHandler(
        OnProcessExit(
            target_action=calibration,
            on_exit=[Shutdown(reason="motion calibration finished")],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "kind",
                default_value="straight",
                choices=["straight", "left", "right"],
            ),
            DeclareLaunchArgument("trial_id", default_value="trial_unspecified"),
            DeclareLaunchArgument("minimum_samples_per_tag", default_value="10"),
            DeclareLaunchArgument("max_observation_wait_s", default_value="60.0"),
            support,
            delayed_calibration,
            stop_support,
        ]
    )
