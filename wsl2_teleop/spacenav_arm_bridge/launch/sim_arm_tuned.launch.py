"""Simulated arm (RViz, fake hardware, no arm needed) with the same model as real_arm_tuned.launch.py:
the real joint limits from the arm repo README are in the URDF, so RViz sliders, MoveIt planning
and the SpaceMouse teleop all stop where the real arm has to.

  ros2 launch spacenav_arm_bridge sim_arm_tuned.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    tuned = os.path.join(get_package_share_directory("spacenav_arm_bridge"), "launch", "real_arm_tuned.launch.py")
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tuned),
        launch_arguments={"ros2_control_plugin": "fake", "readme_limits": "true"}.items(),
    )])
