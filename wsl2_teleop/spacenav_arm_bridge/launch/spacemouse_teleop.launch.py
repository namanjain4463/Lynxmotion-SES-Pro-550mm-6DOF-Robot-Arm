import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    fwd_yaml = os.path.join(get_package_share_directory("spacenav_arm_bridge"),
                            "config", "forward_position_controller.yaml")

    # Load forward_position_controller (inactive) next to the arm's trajectory controller
    spawn = ExecuteProcess(
        cmd=["ros2", "run", "controller_manager", "spawner", "forward_position_controller",
             "-c", "/controller_manager",
             "-t", "position_controllers/JointGroupPositionController",
             "-p", fwd_yaml, "--inactive"],
        output="screen",
    )
    # Hand the arm over from the trajectory controller to the forward controller
    switch = ExecuteProcess(
        cmd=["ros2", "control", "switch_controllers",
             "--deactivate", "arm_trajectory_controller",
             "--activate", "forward_position_controller"],
        output="screen",
    )
    teleop = Node(
        package="spacenav_arm_bridge",
        executable="spacemouse_teleop",
        name="spacemouse_teleop",
        output="screen",
        parameters=[{"linear_speed": 0.05, "angular_speed": 0.0}],
    )

    return LaunchDescription([
        spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[switch])),
        RegisterEventHandler(OnProcessExit(target_action=switch, on_exit=[teleop])),
    ])
