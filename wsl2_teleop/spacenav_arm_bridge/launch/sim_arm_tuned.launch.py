"""Simulated arm (RViz, fake hardware, no arm needed) with the same model as real_arm_tuned.launch.py:
the real joint limits from the arm repo README are in the URDF, so RViz sliders, MoveIt planning
and the SpaceMouse teleop all stop where the real arm has to, and the CGE-10-10 gripper is shown.

  ros2 launch spacenav_arm_bridge sim_arm_tuned.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    tuned = os.path.join(get_package_share_directory("spacenav_arm_bridge"), "launch", "real_arm_tuned.launch.py")
    return LaunchDescription([
        DeclareLaunchArgument("gripper", default_value="cge_1010", choices=["cge_1010", "none"],
                              description="Gripper in the model"),
        DeclareLaunchArgument("finger", default_value="40", choices=["20", "40", "60"],
                              description="Arm repo's CGE finger model; 40 is closest to the installed fingers"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tuned),
            launch_arguments={"ros2_control_plugin": "fake", "readme_limits": "true",
                              "gripper": LaunchConfiguration("gripper"),
                              "finger": LaunchConfiguration("finger")}.items(),
        ),
    ])
