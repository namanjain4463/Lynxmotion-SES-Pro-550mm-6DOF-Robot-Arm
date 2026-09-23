# Running the whole hardware pipeline on native Ubuntu 22.04

The same pipeline as the WSL2 notes ([`README.md`](README.md)): the real Lynxmotion SES-Pro 550
under ROS 2 Humble + MoveIt, SpaceMouse teleop, and the measurement tools. It runs on a PC
booted into Ubuntu 22.04 (dual boot or a separate Linux machine) instead of WSL2.

**Status: written 2026-09-22, not yet run on native Ubuntu.** Every step below is the WSL2
procedure that works, minus the Windows USB forwarding. Section 7 checks that native is actually
better.

Why bother: under WSL2 the arm's USB goes through usbipd (a network tunnel). Measured cost:
the arm driver's loop runs at ~8 Hz instead of 30 Hz, commands arrive ~0.3 s late, the loop
freezes for 0.3–2.9 s about every 40 s, the link dropped about every 1–2 hours, and the
wrist joints move in a stop-go way ([`gate_a_phase0_results.md`](gate_a_phase0_results.md)).
Native USB removes the tunnel.

---

## 1. Install ROS 2 Humble (once)

```bash
sudo apt update && sudo apt install -y locales curl software-properties-common
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-humble-desktop ros-dev-tools python3-colcon-common-extensions python3-rosdep
sudo rosdep init; rosdep update
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```
Open a new terminal and check that `echo $ROS_DISTRO` prints `humble`.

## 2. Packages for the arm, the teleop and the SpaceMouse (once)

```bash
sudo apt install -y ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-position-controllers ros-humble-ros2controlcli ros-humble-xacro \
  spacenavd libspnav-dev ros-humble-spacenav
```

## 3. USB permissions and gotchas (once)

Plug in the arm (USB board STMicroelectronics `0483:5740`) and the SpaceMouse (`256f:c635`).
There is no usbipd: both show up directly.

```bash
lsusb | grep -E "0483:5740|256f:c635"   # both listed
ls -l /dev/ttyACM0                      # the arm's serial port
sudo usermod -aG dialout $USER          # serial port access; log out and back in afterwards
```

**ModemManager** (installed by default on Ubuntu desktop) probes new `/dev/ttyACM*` devices as
if they were modems and can send bytes to the arm's board for several seconds after it is
plugged in. Tell it to leave the arm alone:
```bash
echo 'ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", ENV{ID_MM_DEVICE_IGNORE}="1", SYMLINK+="lynxmotion_arm"' \
  | sudo tee /etc/udev/rules.d/99-lynxmotion-arm.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```
This also creates `/dev/lynxmotion_arm`, a stable name for the arm. The arm's launch files
still use `/dev/ttyACM0`, so keep the arm as the only `ttyACM` device, or plug it in first.

**Real-time scheduling (optional, may reduce loop jitter).** `ros2_control_node` warns
`Could not enable FIFO RT scheduling policy`. To allow it:
```bash
echo "$USER - rtprio 99" | sudo tee /etc/security/limits.d/99-realtime.conf
```
Then log out and back in.

## 4. Build the workspaces (once)

**Arm workspace** (the arm repo, read-only — build it, don't change it):
```bash
cd ~
git clone https://github.com/namanjain4463/Lynxmotion-SES-Pro-550mm-6DOF-Robot-Arm.git
cd Lynxmotion-SES-Pro-550mm-6DOF-Robot-Arm/SES-P-ROS2-Arms
rosdep install --from-paths src/pro_arm_description src/pro_arm_moveit \
  src/pro_hardware_interface src/pro_sim_examples --ignore-src -r -y
colcon build --symlink-install --packages-skip machine_vision_pkg
```

**Teleop workspace** (from this branch of the arm repo; the arm workspace above uses `main`):
```bash
cd ~
git clone -b wsl2-bringup-spacemouse-teleop \
  https://github.com/namanjain4463/Lynxmotion-SES-Pro-550mm-6DOF-Robot-Arm.git lynxmotion_teleop_src
mkdir -p ~/spacemouse_teleop_ws/src
cp -r ~/lynxmotion_teleop_src/wsl2_teleop/spacenav_arm_bridge ~/spacemouse_teleop_ws/src/
cp ~/lynxmotion_teleop_src/wsl2_teleop/tools/*.py ~/
source ~/Lynxmotion-SES-Pro-550mm-6DOF-Robot-Arm/SES-P-ROS2-Arms/install/setup.bash
cd ~/spacemouse_teleop_ws && colcon build --symlink-install
```

## 5. Every session

Three terminals. Unlike WSL2, there is no USB attach step and nothing to do on Windows.

**Terminal 1 — arm** (MoveIt, controllers, RViz). `real_arm_tuned.launch.py` is the arm repo's
`real_arm_control.launch.py` with the README joint limits in the URDF and the wrist servo
acceleration lowered to 30 deg/s² (see the main [README](README.md) section 7):
```bash
source ~/Lynxmotion-SES-Pro-550mm-6DOF-Robot-Arm/SES-P-ROS2-Arms/install/setup.bash
source ~/spacemouse_teleop_ws/install/setup.bash
ros2 launch spacenav_arm_bridge real_arm_tuned.launch.py
```
If the arm has just been powered or plugged in, wait ~5 s before launching (ModemManager rule
aside, the board needs a moment).

**Terminal 2 — SpaceMouse:**
```bash
sudo systemctl restart spacenavd
source /opt/ros/humble/setup.bash
ros2 run spacenav spacenav_node
```

**Terminal 3 — teleop** (loads and switches to `forward_position_controller` by itself):
```bash
source ~/Lynxmotion-SES-Pro-550mm-6DOF-Robot-Arm/SES-P-ROS2-Arms/install/setup.bash
source ~/spacemouse_teleop_ws/install/setup.bash
ros2 launch spacenav_arm_bridge spacemouse_teleop.launch.py
```
Wait for `SpaceMouse teleop ready`. **Hold the left SpaceMouse button for 1 s** = go to the
ready pose; a short press switches between JOINT and TIP mode (all controls: main
[README](README.md) section 7). Ctrl-C stops teleop (the arm holds its position).

**Back to RViz planning mode** (Plan & Execute), in any sourced terminal:
```bash
ros2 control switch_controllers --deactivate forward_position_controller --activate arm_trajectory_controller
```
Only one of the two controllers can drive the arm at a time. With `real_arm_tuned.launch.py`,
RViz planning also respects the README joint limits (the arm repo's own launch allows ±180°).

## 6. If the arm driver stops

Terminal 1 shows `Input/output error` / `Hardware interface error`, and teleop prints
`Teleop paused: arm driver stopped`. On native Ubuntu there is no `wsl --shutdown`:
1. Ctrl-C terminals 3 and 1.
2. Check `ls -l /dev/ttyACM0` and `dmesg | tail` (look for a USB disconnect). If the port is
   gone, unplug and replug the arm's USB cable.
3. Relaunch terminal 1, then terminal 3.

If this still happens regularly on native Ubuntu, the cause is the arm's board, cable or power
supply, not the Windows tunnel. That is useful to know.

## 7. Check that native is better (do this once)

Put the arm in the ready pose (hold the left button), Ctrl-C the teleop (the forward controller stays
active), then:
```bash
source ~/Lynxmotion-SES-Pro-550mm-6DOF-Robot-Arm/SES-P-ROS2-Arms/install/setup.bash
python3 ~/arm_characterize.py all
```
Compare with the WSL2 run ([`data/20260922_180034_all`](data/20260922_180034_all)). What to
look at:

| measure | WSL2 (measured) | expected on native |
|---|---|---|
| control-loop rate | ~8 Hz | close to 30 Hz |
| loop freezes > 0.3 s | 19 in ~12 min | ~0 |
| step delay, joints 1–4 | 0.27–0.30 s | clearly lower |
| moving-target lag | 0.17–0.33 s | clearly lower |
| speed ripple, joints 5–6 | ±1.6–1.7 deg/s at 5 deg/s with the arm's acceleration 100; ±0.4–0.6 with the tuned launch's 30 | lower again, if the slow loop contributes |

Add the new run's folder under `data/` and the numbers to `gate_a_phase0_results.md`. If the
loop is still slow natively, the limit is the servo bus itself (12 request/reply exchanges per
cycle), not WSL2.

## Differences from the WSL2 notes, in one list

- No usbipd, no `wsl --shutdown`, no PowerShell windows, and no bus IDs.
- `dialout` group and the ModemManager udev rule replace the usbipd bind/attach steps.
- SpaceMouse: same `spacenavd` + `spacenav_node`. Restart `spacenavd` if the SpaceMouse was
  plugged in after boot.
- The simulation launch (`sim_arm_tuned.launch.py`, main README section 4) works the same.
- Everything else is identical: the same arm repo build, teleop package, tools, controller
  switching, ready pose and joint-limit behaviour.
