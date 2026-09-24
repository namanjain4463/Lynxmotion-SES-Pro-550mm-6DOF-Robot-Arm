import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
        parameters=[{"linear_speed": ParameterValue(LaunchConfiguration("linear_speed"), value_type=float),
                     "angular_speed": ParameterValue(LaunchConfiguration("angular_speed"), value_type=float),
                     "start_mode": LaunchConfiguration("start_mode"),
                     # calibrated 2026-09-22: both puck tilts were reversed, twist was right
                     "rot_axis_map": [-1, -2, 3]}],
    )

    # DH CGE-10-10 gripper over a USB-RS485 adapter; off by default (no gripper in simulation)
    gripper = Node(
        package="spacenav_arm_bridge",
        executable="gripper_node",
        name="gripper",
        output="screen",
        condition=IfCondition(LaunchConfiguration("gripper")),
        parameters=[{"port": LaunchConfiguration("gripper_port"),
                     "force_pct": ParameterValue(LaunchConfiguration("gripper_force"), value_type=int),
                     "log_dir": LaunchConfiguration("gripper_log_dir")}],
    )

    return LaunchDescription([
        DeclareLaunchArgument("linear_speed", default_value="0.1",
                              description="Tip speed at full puck push, m/s"),
        DeclareLaunchArgument("angular_speed", default_value="0.5",
                              description="Gripper turn speed at full puck tilt/twist, rad/s (0.5 = ~29 deg/s)"),
        DeclareLaunchArgument("start_mode", default_value="joint",
                              description="joint = each puck motion drives one joint, tip = straight-line tip moves"),
        DeclareLaunchArgument("gripper", default_value="false",
                              description="true = start the gripper node (right SpaceMouse button toggles it)"),
        DeclareLaunchArgument("gripper_port", default_value="/dev/ttyUSB0",
                              description="Serial port of the USB-RS485 adapter"),
        DeclareLaunchArgument("gripper_force", default_value="20", description="Grip force 20-100 %"),
        DeclareLaunchArgument("gripper_log_dir", default_value="",
                              description="Folder for a CSV log of the gripper state ('' = no log)"),
        gripper,
        spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[switch])),
        RegisterEventHandler(OnProcessExit(target_action=switch, on_exit=[teleop])),
    ])
