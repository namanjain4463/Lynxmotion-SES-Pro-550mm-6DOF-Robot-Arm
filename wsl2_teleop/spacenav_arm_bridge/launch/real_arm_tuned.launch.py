"""Real-arm launch identical to pro_arm_moveit/real_arm_control.launch.py, except (the arm repo
itself is not changed):
- servo acceleration of joints 5-6. Measured 2026-09-22: at the arm's default 100 deg/s^2 they
  move in a jerky way (+-1.3-2.0 deg/s ripple at 5 deg/s); at 30, like joints 1-3, +-0.4-0.6.
- the URDF joint limits are set to the real ones from the arm repo README (the URDF has +-180 deg
  everywhere), so RViz/MoveIt planning respects them too.
- the CGE-10-10 gripper is in the model by default (it is physically mounted): RViz shows it,
  MoveIt's collision checks include it, and the tool point `pro_arm_ee` is the gripper tip. Its
  ros2_control joint (joint_7) is simulated even with the real arm (the arm repo has no gripper
  driver); gripper_node mirrors the real finger position onto it, so RViz follows the real gripper.
  finger:=40 is the closest of the arm repo's finger models to the installed fingers (model ~35-46 mm
  open, real ~52 mm); its finger:=60 model is asymmetric. gripper:=none restores the old model.

  ros2 launch spacenav_arm_bridge real_arm_tuned.launch.py              # wrist 30, README limits
  ros2 launch spacenav_arm_bridge real_arm_tuned.launch.py wrist_acceleration:=100 readme_limits:=false
  ros2 launch spacenav_arm_bridge real_arm_tuned.launch.py ros2_control_plugin:=fake   # simulation
  ros2 launch spacenav_arm_bridge real_arm_tuned.launch.py gripper:=none               # no gripper model
  (sim_arm_tuned.launch.py does the last one)
"""
import math
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# Real joint limits (deg) from the arm repo README; joint 6 is untested there, so left as in the URDF
README_LIMITS_DEG = {1: (-180.0, 180.0), 2: (-90.0, 90.0), 3: (-115.0, 115.0), 4: (-130.0, 160.0), 5: (-105.0, 180.0)}

# Same xacro arguments the arm's own launch resolves to on this arm (6 DOF, 550 mm); gripper and
# finger are set per launch
XACRO_ARGS = {"name": "pro_arm", "prefix": "pro_arm_", "dof": "6", "size": "550", "gripper": "cge_1010",
              "finger": "40", "ros2_control": "true", "ros2_control_plugin": "real"}


def tuned_urdf(accel_by_joint, limits_deg=None, plugin="real", gripper="cge_1010", finger="40"):
    src = os.path.join(get_package_share_directory("pro_arm_description"), "urdf", "pro_arm.urdf.xacro")
    doc = xacro.process_file(src, mappings=dict(XACRO_ARGS, ros2_control_plugin=plugin,
                                                gripper=gripper, finger=finger))
    done = set()
    # Only touch <param name="acceleration"> of <joint> elements INSIDE the arm's <ros2_control> block;
    # the same joint names also appear elsewhere in the URDF.
    for block in doc.getElementsByTagName("ros2_control"):
        for joint in block.getElementsByTagName("joint"):
            name = joint.getAttribute("name")
            for num, accel in accel_by_joint.items():
                if name != f"pro_arm_joint_{num}":
                    continue
                for param in joint.getElementsByTagName("param"):
                    if param.getAttribute("name") == "acceleration":
                        param.firstChild.data = str(accel)
                        done.add(num)
    missing = set(accel_by_joint) - done
    if missing:
        raise RuntimeError(f"could not find the acceleration setting of joint(s) {sorted(missing)}")
    # Kinematic joints (the <joint> elements directly under <robot>) carry the <limit> MoveIt uses
    for num, (lo, hi) in (limits_deg or {}).items():
        found = False
        for joint in doc.documentElement.getElementsByTagName("joint"):
            if joint.parentNode is doc.documentElement and joint.getAttribute("name") == f"pro_arm_joint_{num}":
                for limit in joint.getElementsByTagName("limit"):
                    limit.setAttribute("lower", repr(math.radians(lo)))
                    limit.setAttribute("upper", repr(math.radians(hi)))
                    found = True
        if not found:
            raise RuntimeError(f"could not find the URDF limit of joint {num}")
    return doc.toprettyxml(indent="  ")


def setup(context):
    wrist = LaunchConfiguration("wrist_acceleration").perform(context)
    use_limits = LaunchConfiguration("readme_limits").perform(context).lower() in ("true", "1", "yes")
    plugin = LaunchConfiguration("ros2_control_plugin").perform(context)
    gripper = LaunchConfiguration("gripper").perform(context)
    finger = LaunchConfiguration("finger").perform(context)
    if plugin not in ("real", "fake"):
        raise RuntimeError(f"ros2_control_plugin must be 'real' or 'fake', not '{plugin}'")
    accel = {5: int(float(wrist)), 6: int(float(wrist))}
    path = f"/tmp/pro_arm_tuned_{os.getpid()}.urdf"
    with open(path, "w") as f:
        f.write(tuned_urdf(accel, README_LIMITS_DEG if use_limits else None, plugin, gripper, finger))
    print(f"[real_arm_tuned] hardware: {plugin}; gripper model: {gripper} (finger {finger}); servo acceleration joint 5 = {accel[5]}, joint 6 = {accel[6]} deg/s^2 "
          f"(joints 1-4 unchanged); README joint limits in the URDF: {'yes' if use_limits else 'no'}; "
          f"URDF written to {path}")
    move_arm = os.path.join(get_package_share_directory("pro_arm_moveit"), "launch", "move_arm.launch.py")
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(move_arm),
        # an absolute description_filepath replaces the package path, so our generated file is used
        launch_arguments={"dof": "6", "size": "550", "ros2_control_plugin": plugin,
                          "gripper": gripper, "finger": finger,
                          "description_filepath": path}.items(),
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("wrist_acceleration", default_value="30",
                              description="Servo acceleration for joints 5 and 6 (arm default 100)"),
        DeclareLaunchArgument("readme_limits", default_value="true",
                              description="Put the README joint limits into the URDF (RViz/MoveIt then respect them)"),
        DeclareLaunchArgument("ros2_control_plugin", default_value="real",
                              description="'real' = the arm over USB, 'fake' = simulation in RViz (no arm needed)"),
        DeclareLaunchArgument("gripper", default_value="cge_1010", choices=["cge_1010", "none"],
                              description="Gripper in the model (RViz, collisions, tool point)"),
        DeclareLaunchArgument("finger", default_value="40", choices=["20", "40", "60"],
                              description="Arm repo's CGE finger model; 40 is closest to the installed fingers"),
        OpaqueFunction(function=setup),
    ])
