# Source this in every terminal used for the Lynxmotion arm / gripper / SpaceMouse:
#     source ~/ros_arm_env.sh
# ROS 2 Humble needs the system Python 3.10, so any conda env (mjwarp, env_isaaclab, base)
# is deactivated first. Do not run Isaac Sim's ROS bridge at the same time (same ROS_DOMAIN_ID 0).
while [ -n "${CONDA_PREFIX:-}" ] && command -v conda >/dev/null 2>&1; do conda deactivate || break; done
source /opt/ros/humble/setup.bash
source "$HOME/Lynxmotion-SES-Pro-550mm-6DOF-Robot-Arm/SES-P-ROS2-Arms/install/setup.bash"
source "$HOME/spacemouse_teleop_ws/install/setup.bash"
# Stable device names from /etc/udev/rules.d/99-lynxmotion-arm.rules and 99-cge-gripper.rules
export GRIPPER_PORT=/dev/cge_gripper
echo "ROS $ROS_DISTRO ready | python $(python3 -c 'import sys;print(sys.version.split()[0])') | arm: $( [ -e /dev/ttyACM0 ] && echo /dev/ttyACM0 || echo 'not plugged') | gripper: $( [ -e $GRIPPER_PORT ] && echo $GRIPPER_PORT || echo 'not plugged')"
