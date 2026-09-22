# Lynxmotion SES-Pro 550 on Windows 11 + WSL2: bring-up, measured limits, SpaceMouse teleop

Field notes from bringing up the real SES-Pro 550 mm 6-DOF arm and a 3Dconnexion SpaceMouse
Compact on a Windows 11 laptop running ROS 2 Humble inside WSL2 (September 2026). Everything
below was run on the real hardware; numbers are measured, not estimated.

This folder is kept identical in two places:
- arm repo, branch `wsl2-bringup-spacemouse-teleop`:
  [`wsl2_teleop/`](https://github.com/namanjain4463/Lynxmotion-SES-Pro-550mm-6DOF-Robot-Arm/tree/wsl2-bringup-spacemouse-teleop/wsl2_teleop)
- Thesis repo: `hardware/lynxmotion_wsl2_teleop/`

Contents:
- [`spacenav_arm_bridge/`](spacenav_arm_bridge) — ROS 2 package with the SpaceMouse teleop node and launch file
- [`tools/`](tools) — small measurement scripts used to find the problems described here

---

## 1. Status in one paragraph

MoveIt Plan & Execute drives the real arm from WSL2. SpaceMouse Cartesian teleop (tip
translation) works through a custom node plus a `forward_position_controller`, and the tip
follows the commanded velocity to within ~5% on every axis. The main limitation is the
Windows → WSL2 USB forwarding (usbipd): the arm driver's control loop only reaches **~9 Hz
instead of 30 Hz**, commands reach the servos **~0.35 s late**, the link occasionally
**freezes for ~3 s**, and it sometimes **drops completely** until WSL is restarted. MoveIt
Servo does not work on this arm (section 6). For the final thesis setup, run ROS on native
Ubuntu 22.04 instead of WSL2.

---

## 2. Setup used

| Item | Value |
|---|---|
| Host | Windows 11 laptop, WSL2 Ubuntu 22.04, ROS 2 Humble |
| USB forwarding | usbipd-win 5.3.0 (`winget install usbipd`) |
| Arm USB board | STMicroelectronics Virtual COM Port, VID:PID `0483:5740` → `/dev/ttyACM0` in WSL |
| SpaceMouse | 3Dconnexion SpaceMouse Compact, VID:PID `256f:c635` |
| SpaceMouse driver | `spacenavd` 0.7.1 + `ros-humble-spacenav` 3.3.0 (`/spacenav/twist`, ~31–35 Hz even at rest) |
| Arm workspace | `SES-P-ROS2-Arms/` from the arm repo, built with `--packages-skip machine_vision_pkg` |
| Teleop workspace | separate overlay, e.g. `~/spacemouse_teleop_ws/src/spacenav_arm_bridge` |

---

## 3. One-time setup

**Windows, PowerShell (admin)** — share both devices with WSL (persists across reboots):
```powershell
usbipd bind --hardware-id 0483:5740
usbipd bind --hardware-id 256f:c635
```
Use `--hardware-id`, not `--busid`: bus IDs changed several times during testing after USB
resets/replugs. Never attach your own keyboard/mouse (it disappears from Windows).

**WSL** — permissions and packages:
```bash
groups   # must include dialout (sudo usermod -aG dialout $USER, then log out/in)
sudo apt install -y spacenavd libspnav-dev ros-humble-spacenav \
                    ros-humble-position-controllers ros-humble-ros2controlcli
```

**WSL** — build the arm workspace (only the packages needed; the vision package is skipped):
```bash
git clone https://github.com/namanjain4463/Lynxmotion-SES-Pro-550mm-6DOF-Robot-Arm.git
cd Lynxmotion-SES-Pro-550mm-6DOF-Robot-Arm/SES-P-ROS2-Arms
rosdep install --from-paths src/pro_arm_description src/pro_arm_moveit \
  src/pro_hardware_interface src/pro_sim_examples --ignore-src -r -y
colcon build --symlink-install --packages-skip machine_vision_pkg
```

**WSL** — build the teleop overlay (copy `spacenav_arm_bridge/` from this folder):
```bash
mkdir -p ~/spacemouse_teleop_ws/src
cp -r spacenav_arm_bridge ~/spacemouse_teleop_ws/src/
cd ~/spacemouse_teleop_ws && colcon build --symlink-install
```

---

## 4. Start-up procedure (every session)

Order matters: do all USB attaching **before** starting ROS, and never run `usbipd` commands
while the arm launch is running.

1. **PowerShell (admin) window A** — arm (leave open; re-attaches automatically if the link drops):
   ```powershell
   usbipd attach --wsl --hardware-id 0483:5740 --auto-attach
   ```
2. **PowerShell (admin) window B** — SpaceMouse (leave open):
   ```powershell
   usbipd attach --wsl --hardware-id 256f:c635 --auto-attach
   ```
3. **WSL terminal 1** — arm (MoveIt + controllers + RViz):
   ```bash
   lsusb    # must list STMicroelectronics Virtual COM Port and 3Dconnexion SpaceMouse
   source ~/Lynxmotion-SES-Pro-550mm-6DOF-Robot-Arm/SES-P-ROS2-Arms/install/setup.bash
   ros2 launch pro_arm_moveit real_arm_control.launch.py
   ```
4. **RViz** — the arm is now in *planning mode* (`arm_trajectory_controller`). Before teleop,
   move it off the all-zero pose into a bent pose, e.g. joint_2 = −25°, joint_3 = −25°,
   joint_5 = −20° (Joints tab → Planning tab → Plan & Execute). The all-zero pose stands the
   arm straight up, which is a kinematic singularity. The SRDF's zero pose is named
   **`default`** (there is no `home`).
5. **WSL terminal 2** — SpaceMouse (restart the daemon if the device was attached after boot):
   ```bash
   sudo systemctl restart spacenavd
   source ~/Lynxmotion-SES-Pro-550mm-6DOF-Robot-Arm/SES-P-ROS2-Arms/install/setup.bash
   ros2 run spacenav spacenav_node
   ```
6. **WSL terminal 3** — teleop (loads and switches to the forward controller automatically):
   ```bash
   source ~/Lynxmotion-SES-Pro-550mm-6DOF-Robot-Arm/SES-P-ROS2-Arms/install/setup.bash
   source ~/spacemouse_teleop_ws/install/setup.bash
   ros2 launch spacenav_arm_bridge spacemouse_teleop.launch.py
   ```
   Expect `Successfully switched controllers` and `SpaceMouse teleop ready`. A spawner
   warning `Controller already loaded` is normal on the second run.
7. **Back to planning mode** (RViz Plan & Execute) — Ctrl-C terminal 3, then:
   ```bash
   ros2 control switch_controllers --deactivate forward_position_controller --activate arm_trajectory_controller
   ```

**Only one thing may command the arm at a time.** RViz Plan & Execute uses
`arm_trajectory_controller`; teleop uses `forward_position_controller`. They cannot both be
active (they claim the same joint interfaces).

**Recovery** — if the arm log shows `Input/output error`, or the spawners print
`Could not contact service /controller_manager/list_controllers`: Ctrl-C every ROS terminal,
run `wsl --shutdown` in PowerShell, then repeat steps 1–3. Relaunching alone does not help
(see section 5.2).

---

## 5. Measured numbers and limitations

### 5.1 Speed and delay (measured with the scripts in `tools/`)

| Measurement | Result |
|---|---|
| Driver control loop (`ros2 topic hz /joint_states`, configured 30 Hz) | **~9 Hz** (periods 0.093–0.133 s), plus occasional **~3 s gaps** |
| Single trajectory, base 21.8° commanded in 2.0 s | reached in 2.53 s |
| Single trajectory, base 20° commanded in 1.0 s | reached in 1.81 s, peak ~28°/s |
| Delay before any motion starts (single trajectory) | ~0.4–0.5 s |
| Forward controller, target streamed at 5°/s | arm follows *during* the stream, ~0.35 s behind (1.5–1.9°), reaches 100% |
| Teleop tip tracking, 6 s push | commanded x −6.6 / y −11.7 / z −4.5 cm → actual −6.6 / −11.4 / −4.3 cm |
| Teleop tip tracking, 11 s push | commanded x +20.9 / y +25.9 cm → actual +20.3 / +24.5 cm |

The arm repo README states the arm "physically tops out at ≈ 5°/s". That is not a hardware
limit: single moves measured 10–28°/s.

**Why the loop is only ~9 Hz:** every control cycle the driver
(`pro_hardware_interface/pro_motor_hardware`) does, for each of the 6 servos, a position query
and a speed query (12 request/reply exchanges) plus a move command, each with a 1 ms sleep.
Under usbipd every exchange is a round trip through the Windows↔WSL network tunnel, so a cycle
takes ~110 ms. This, plus the servos' own response, is where the ~0.35 s delay comes from.
Options: native Ubuntu (removes the tunnel), or a patched copy of the driver in an overlay
workspace that derives velocity from position instead of querying it (halves the traffic,
expected ~15 Hz; not done yet).

### 5.2 The USB link drops

Seen several times, usually a few minutes into a session and often while a move is executing:
- The arm log shows `[ProMotorHardware]: Failed to read position/speed from motor N: Input/output error`,
  then `Hardware interface error`, then the same error for all six motors during error handling.
- `Input/output error` here is the driver's own label, not necessarily a kernel USB error:
  `pro.c` waits 100 ms (`VTIME=1`) for a servo reply, retries 3 times, then sets `errno=EIO`.
  All six servos going silent at once (LEDs still steady blue) means the PC↔USB-board link
  died, not the servos.
- The driver's `on_error()` calls `cleanup()`, which **closes the serial port**; that launch
  can no longer move the arm.
- Windows `usbipd list` then shows the arm as **Shared** instead of **Attached**, while WSL
  still shows `/dev/ttyACM0` and `dmesg` shows no disconnect. The next launch opens this dead
  port and hangs during hardware init: the spawners print `Could not contact service
  /controller_manager/list_controllers` and MoveIt reports `couldn't receive full current
  joint state within 1s`.
- Fix: `wsl --shutdown`, reattach, relaunch. Root cause not pinned down (USB/IP tunnel vs.
  the arm board resetting). Windows' `Microsoft-Windows-Kernel-PnP/Configuration` event log
  showed no `VID_0483` re-enumeration at the failure times.

### 5.3 Other gotchas

- **WSL2 needs usbipd for each USB device.** `bind` persists; `attach` is lost on every WSL
  restart, unplug or link drop. The arm's USB board is a separate device from the SpaceMouse.
- **MoveIt trajectory timeouts cannot be changed at runtime.** `ros2 param set /move_group
  trajectory_execution.allowed_execution_duration_scaling ...` reports success but has no effect.
- **`arm_trajectory_controller` cannot follow a stream of short trajectories.** When a new
  0.1 s trajectory arrives every 0.1 s, the arm does not move at all until the stream stops:
  each new message restarts the motion from the measured position, and the servos need
  ~0.4 s to get going. A single longer trajectory works fine. Anything that streams targets
  (teleop, servoing) needs `forward_position_controller` instead.
- **RViz STOP button (stock `emergency_stop_marker.py`)** is a toggle that deactivates /
  re-activates `arm_trajectory_controller` only, so in teleop mode it has no effect on its own.
  The teleop node listens to the same button (`/emergency_stop_button/feedback`) and pauses
  itself — implemented, not yet verified on the hardware. The E-stop node also publishes to
  `/pro_arm_controller/...` topics that nothing subscribes to (harmless).
- **Harmless warnings** at arm launch are listed in the arm repo README; add to them an
  occasional `Failed to receive current joint state` (the slow 9 Hz loop).

---

## 6. Why MoveIt Servo does not work on this arm

`moveit_servo` 2.5.10 (Humble) was tried first and dropped:
1. It starts paused — needs `ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}`
   after every start.
2. With the example (Panda) singularity thresholds (17/30) it reports
   `HALT_FOR_SINGULARITY` (status 2) in the all-zero pose and in mildly bent poses; raising them
   (100/200) removed the halt.
3. **The blocker:** Servo computes every new target from the *measured* joint position. With
   the arm's ~0.35 s response delay the target never gets ahead of the arm. Measured with
   `tools/arm_timer.py` and `tools/fwd_test.py` (5°/s requested for 4 s):

   | Controller | Target advances from its own previous target | Target recomputed from measured position (Servo's method) |
   |---|---|---|
   | `arm_trajectory_controller`, 0.1 s trajectories | arm frozen during the stream, catches up after | 2% |
   | `forward_position_controller`, 30 Hz | follows, ~0.35 s behind, 100% | 8% |

   Consistent with this, during a real Servo run the command stayed a fixed 3.5° / 6.1° / 2.6°
   ahead on joints 2/3/5 for 8 s while the arm moved ~0°.

`servo_config.yaml` and `twist_bridge.py` are kept in the package only as the record of that
attempt; the teleop launch file does not use them.

---

## 7. The teleop node (`spacenav_arm_bridge/spacemouse_teleop.py`)

How it works:
1. While idle it reads the tip pose of the real arm with MoveIt's `/compute_fk` (every 1 s
   after 1 s of no input), so a push starts immediately and never jumps.
2. While the puck is pushed it moves **its own** tip target at `linear_speed × puck value`
   (open loop — it does not restart from the measured position, see section 6).
3. Each step it solves the new tip pose with MoveIt's `/compute_ik` (same KDL solver and
   collision checking as RViz planning), seeded with the previous solution, and publishes the
   joint angles to `/forward_position_controller/commands` at 30 Hz.
4. Safety: if IK fails (reach, joint limit, collision) it does not move and warns
   `Can't go further that way`; if the solution jumps more than `flip_threshold` in one step
   (IK switching arm configuration) it refuses; if joints would exceed `max_joint_speed` it
   takes a proportionally shorter step instead of stopping. RViz STOP pauses it (see 5.3).

Parameters (set in `launch/spacemouse_teleop.launch.py`):

| Parameter | Default | Meaning |
|---|---|---|
| `linear_speed` | 0.05 | tip speed in m/s at puck value 1.0 (typical pushes read 0.4–0.7) |
| `angular_speed` | 0.0 | tip rotation in rad/s at puck value 1.0; 0 = translation only |
| `deadband` | 0.05 | puck values below this are ignored |
| `axis_map` | [1, 2, 3] | tip x, y, z ← puck axis number (1 = x, 2 = y, 3 = z); negative = reversed |
| `rate` | 30.0 | Hz |
| `max_joint_speed` | 0.6 | rad/s cap per joint |
| `flip_threshold` | 0.1 | rad per step treated as an IK configuration flip |
| `resync_after` | 1.0 | s of no input before re-reading the real pose |

Observed behaviour and open items:
- With `axis_map = [1, 2, 3]`, puck x/y/z moves the tip along the arm base frame's x/y/z with
  the same sign; up/down is correct. Aligning away/toward and left/right with the operator's
  seat is still to be calibrated through `axis_map`.
- The puck reports small vertical values during sideways pushes, so the tip slowly sinks on
  long pushes (~3 cm over 11 s). A larger `deadband` (~0.1) should remove this.
- Some pushes stall at their start with `Can't go further`; not yet diagnosed (IK timeout of
  20 ms vs. collision model vs. reach). Planned: retry with a longer timeout and report which.
- Motion lags the puck by ~0.35–0.45 s (section 5.1).
- Tip rotation (`angular_speed > 0`) is untested.

---

## 8. Measurement tools (`tools/`)

Run in a WSL terminal with the arm workspace sourced and the arm launch running.

| Script | Use | Needs |
|---|---|---|
| `arm_timer.py move J TARGET_RAD SECONDS` | one timed move of joint J, prints progress and actual time | planning mode (no teleop) |
| `arm_timer.py stream J DEG_PER_S SECONDS open\|anchored` | streams 0.1 s trajectories; `anchored` mimics MoveIt Servo | planning mode |
| `arm_timer.py watch SECONDS` | prints each joint's change every 0.5 s | any |
| `arm_timer.py servo SECONDS` | MoveIt Servo command vs. actual ("lead") | Servo running |
| `fwd_test.py J DEG_PER_S SECONDS open\|anchored` | same stream test through `forward_position_controller` | forward controller active |
| `tip_watch.py SECONDS` | puck values and tip displacement (cm, base frame) every 0.5 s | any; uses `/compute_fk` |

All moves are limited to 0.6 rad per command. Keep the arm's reach clear when running them.
