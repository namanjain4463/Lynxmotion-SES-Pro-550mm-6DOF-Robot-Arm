# Lynxmotion SES-Pro 550 on Windows 11 + WSL2: bring-up, measured limits, SpaceMouse teleop

Field notes from bringing up the real SES-Pro 550 mm 6-DOF arm and a 3Dconnexion SpaceMouse
Compact on a Windows 11 laptop running ROS 2 Humble inside WSL2 (September 2026). Everything
below was run on the real hardware; numbers are measured, not estimated.

Where this folder lives (two copies, kept identical; only this paragraph, links to Thesis-only
files and the clone step of the native guide differ):
- Thesis repo (`namanjain4463/Thesis`, private) `hardware/lynxmotion_wsl2_teleop/` — the
  maintained copy.
- **This folder: arm repo, branch `wsl2-bringup-spacemouse-teleop`, `wsl2_teleop/`** — public
  mirror, last synced 2026-09-23 (teleop v5 + CGE-10-10 gripper, gripper in RViz). The arm repo's
  `main` branch is not changed.

The package in both is identical to the one tested on the hardware.

Contents:
- [`spacenav_arm_bridge/`](spacenav_arm_bridge) — ROS 2 package: the SpaceMouse teleop node, the
  DH CGE-10-10 gripper node with grasp/slip detection, the tuned real-arm launch (README joint
  limits, calmer wrist servos) and the matching simulation launch
- [`gripper_cge_10_10.md`](gripper_cge_10_10.md) — the gripper: wiring, register map, driver,
  grasp/slip/loss events, measured aperture and behaviour
- [`tools/`](tools) — measurement scripts used to find the problems described here, and the
  standalone gripper test tool
- [`native_ubuntu_setup.md`](native_ubuntu_setup.md) — the same pipeline on native Ubuntu 22.04
  (no usbipd; recommended for the final setup)
- [`gate_a_phase0_results.md`](gate_a_phase0_results.md) and [`data/`](data) — measured servo
  response, noise, backlash, speed limits and jitter (Gate A Phase 0, unloaded)

---

## 1. Status in one paragraph

MoveIt Plan & Execute drives the real arm from WSL2. SpaceMouse teleop works through a custom
node plus a `forward_position_controller`, in two modes switched with a SpaceMouse button:
**JOINT mode** (each puck motion drives one joint, so every pose inside the joint limits can be
reached) and **TIP mode** (the tip moves in straight lines, tilts point the gripper, twist spins
it); in TIP mode the tip follows the commanded velocity to within ~5% on every axis. Both modes,
the button actions, the joint limits and the speeds were tested on the real arm on 2026-09-22
and judged "working very good" by the operator (section 7). The README joint limits are
enforced in teleop and, with this folder's launch file, in RViz/MoveIt planning too. The
**CGE-10-10 gripper** works from ROS since 2026-09-23 through its own USB-RS485 adapter (the arm
repo has no gripper driver): the right SpaceMouse button toggles it, and the node reports catches,
slip, losses and hold times ([`gripper_cge_10_10.md`](gripper_cge_10_10.md)); it has not yet been
used together with the real arm (the laptop has two USB ports; a hub is needed). The main
limitation is the
Windows → WSL2 USB forwarding (usbipd): the arm driver's control loop only reaches **~9 Hz
instead of 30 Hz**, commands reach the servos **~0.35 s late**, the link occasionally
**freezes for ~3 s**, and it sometimes **drops completely** until WSL is restarted. MoveIt
Servo does not work on this arm (section 6). For the final thesis setup, run ROS on native
Ubuntu 22.04 instead of WSL2: [`native_ubuntu_setup.md`](native_ubuntu_setup.md).

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
   With the gripper, a third window (see [`gripper_cge_10_10.md`](gripper_cge_10_10.md); arm,
   SpaceMouse and gripper together need a USB hub on a two-port laptop):
   ```powershell
   usbipd attach --wsl --hardware-id 1a86:7523 --auto-attach
   ```
3. **WSL terminal 1** — arm (MoveIt + controllers + RViz):
   ```bash
   ls -l /dev/ttyACM0    # the arm's serial port must exist
   source ~/Lynxmotion-SES-Pro-550mm-6DOF-Robot-Arm/SES-P-ROS2-Arms/install/setup.bash
   source ~/spacemouse_teleop_ws/install/setup.bash
   ros2 launch spacenav_arm_bridge real_arm_tuned.launch.py
   ```
   This is the arm repo's `real_arm_control.launch.py` (which still works) with three changes
   made on the fly, without touching the arm repo: the README joint limits are written into the
   URDF, so RViz sliders and MoveIt planning respect them; joints 5–6 get servo acceleration
   30 deg/s² instead of 100 (smoother; section 7.5); and the **CGE-10-10 gripper is in the model**
   (since 2026-09-23; it is physically mounted), so RViz shows it, MoveIt's collision checks
   include it, and the tool point `pro_arm_ee` is the gripper tip, ~12 cm beyond the wrist flange
   of the old gripper-less model. The model's gripper joint (`joint_7`) is simulated even with the
   real arm; `gripper_node` mirrors the real finger position onto it
   ([`gripper_cge_10_10.md`](gripper_cge_10_10.md)). Expect `hardware: real`, `gripper model:
   cge_1010 (finger 40)` and `README joint limits in the URDF: yes` in the output. Options:
   `wrist_acceleration:=100`, `readme_limits:=false`, `gripper:=none`, `finger:=20|40|60`.
4. **Ready pose** — the all-zero pose stands the arm straight up, which is a kinematic
   singularity (section 5.3); the SRDF calls it **`default`** (there is no `home`). Teleop needs
   the **ready pose joint_2 = −50°, joint_3 = −60°, joint_5 = −40°** (others 0). Once teleop runs
   (step 6), hold the left SpaceMouse button for 1 s and the arm goes there by itself. Without
   teleop: RViz Planning tab → Velocity Scaling 0.2, Joints tab → set the joints, Plan, Execute.
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
   Expect `Successfully switched controllers`, `SpaceMouse teleop ready` and the mode line.
   On a second run in the same arm session the spawner prints `Controller already loaded` or
   `Failed to configure controller`; both are harmless (the controller is already there).
   Launch options: `start_mode:=tip`, `linear_speed:=...`, `angular_speed:=...` (section 7);
   gripper: `gripper:=true gripper_log_dir:=$HOME/gripper_logs` (also `gripper_port:=...`,
   `gripper_force:=...`), which starts the gripper node before the controller switch.
7. **Back to planning mode** (RViz Plan & Execute) — Ctrl-C terminal 3, then:
   ```bash
   ros2 control switch_controllers --deactivate forward_position_controller --activate arm_trajectory_controller
   ```

**Simulation (no arm needed)** — same model, same README joint limits, fake hardware that follows
every command instantly. Use it to try new teleop settings before the real arm. Skip steps 1 and
4 (keep the SpaceMouse attached with step 2) and replace step 3's launch line with:
```bash
ros2 launch spacenav_arm_bridge sim_arm_tuned.launch.py
```
Steps 5–7 are unchanged. The simulation shows which joints move and in which direction, and where
the limits stop them; it does not show the real arm's ~0.3 s servo delay, the ~9 Hz loop or its
freezes, so the real arm feels less smooth than the simulation. (The arm repo also has an
Ignition Gazebo launch, `sim_arm_control.launch.py`; it adds gravity but not the servo delay or
the serial loop, so it was not needed here.)

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
- **Kinematics (forward kinematics computed offline from the arm's own URDF):**
  - All joints at 0 (`default`) = arm straight up, fully stretched: exactly singular
    (Jacobian condition number ~10²³). The tip cannot move sideways or up from there at all;
    Cartesian teleop and `axis_calib.py` fail in that pose.
  - Joints 2, 3 and 5 rotate about parallel axes, so the arm swings in one vertical plane that
    sits **12.5 cm to the side** of the base axis (link offsets). Moving the tip along the arm's
    x axis needs base + wrist rotation (3 cm ≈ 0.28 rad of joint motion even in a good pose),
    and the tip can never get closer than 12.5 cm to the base axis.
  - Working pose joint_2 = −50°, joint_3 = −60°, joint_5 = −40° (others 0): tip ~40 cm above
    the base (gripper-less model; with the gripper model, default since 2026-09-23, the tool point
    is the gripper tip at ~30 cm), condition number 19, all six ±3 cm moves solvable. Within the range the arm
    reached safely during teleop (about −49° / −70° / −66°).
- **The arm repo's URDF gives every joint limits of ±180°.** The real physical limits in the arm
  repo README (J1 ±180°, J2 ±90°, J3 ±115°, J4 −130…+160°, J5 −105…+180°, J6 not stated, J2/J3
  coupled) are not in the model. `real_arm_tuned.launch.py` / `sim_arm_tuned.launch.py` write
  them into the URDF at launch, so RViz and MoveIt planning respect them; the teleop node
  enforces them itself (minus a 5° margin) whichever launch is used. The J2/J3 coupling is not
  enforced anywhere (the README gives no formula).

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

Version tested on the real arm on 2026-09-22 (evening), after the same checks in simulation.
Operator verdict: speeds "seem perfect" (simulation), then on the arm "everything is working very good".

### 7.1 Controls

SpaceMouse Compact, two buttons (`/spacenav/joy` index 0 = right, 1 = left):

| Button | Short press | Hold 1 s |
|---|---|---|
| **Left** | switch JOINT ↔ TIP mode (the terminal prints the new mode) | go to the ready pose |
| **Right** | gripper open/close (needs the teleop launch with `gripper:=true`) | precision mode on/off (all speeds ×0.3) |

Any button press while the arm is moving to the ready pose stops that move. The mode and the
precision setting are printed every time they change.

**JOINT mode** (the start mode): one joint moves at a time — the strongest puck motion wins, and
another motion has to be 1.3× stronger to take over, so the unavoidable mixing of puck axes
never moves a second joint. The pairing follows what the operator sees from the seat used here
(the arm reaches toward the operator's left at the ready pose):

| Puck motion | Joint | Top speed |
|---|---|---|
| twist | J1 base turn | 25°/s |
| push left/right (along the arm's reach) | J2 shoulder | 25°/s |
| lift/press | J3 elbow | 25°/s |
| push away/toward | J4 forearm roll | 40°/s |
| tilt left/right | J5 wrist bend | 40°/s |
| tilt away/toward | J6 gripper roll | 40°/s |

Each joint's direction is computed at start-up from the Jacobian at the ready pose: the joint
moves the tip (or turns the gripper) the same way as the hand moves the puck there. All six
directions were confirmed on the arm. Directions stay fixed afterwards (it is a joint mode), so
far from the ready pose a joint can look "reversed" relative to the hand.

**TIP mode**:

| Puck motion | What moves |
|---|---|
| push/pull, left/right, up/down | the tip, in straight lines (all joints together), up to 10 cm/s |
| tilt | the gripper points up/down/left/right; only joints 4–6 move (the tip swings a few cm) |
| twist | the gripper spins about its own axis (joint 6 only) |

Push and rotation never mix: whichever the puck does more wins (`one_mode_at_a_time`).

**Response curve (both modes):** after the deadband, output = 0.2·s + 0.8·s³ of full speed
(s = puck deflection, 0–1), so half deflection gives ~14% speed and full deflection 100%:
fine positioning and fast travel without changing settings. `spacenav_node` scales by 512 but the
SpaceMouse Compact only reaches ~350 (~0.68), so a reading of 0.65 counts as full deflection
(`full_deflection`); before this, full speed was never reached.

### 7.2 How it works

1. Kinematics come from the arm's own URDF (`/robot_description`), in
   [`arm_kinematics.py`](spacenav_arm_bridge/spacenav_arm_bridge/arm_kinematics.py): forward
   kinematics and the geometric Jacobian (checked against finite differences to 3e-7).
2. Every step (30 Hz) starts from the last *commanded* joints (open loop — restarting from the
   measured position loses over 90% of the motion, section 6).
3. JOINT mode adds the scaled puck input to one joint. TIP mode solves the joint motion with
   **damped least squares**: the tip orientation is held softly (`rot_weight`), so near the edge
   of reach the position keeps following and the wrist gives a little; a step whose tip motion
   would point more than ~45° away from the push is shrunk or dropped, so a blocked direction
   does not drift sideways; near a singularity the arm slows instead of jumping (no IK "flips").
   Tilts lock joints 1–3 (`rotate_about: wrist`); the alternative `tip` turns the gripper about
   the tip point with the whole arm, which the operator found hard to control.
4. The real joint limits from the arm repo README (minus `limit_margin_deg`) are enforced in
   both modes: a joint that would cross one stops (JOINT mode) or is locked while the others take
   over (TIP mode), with a "joint_N at its limit" message; moves back out are always allowed.
5. Every step is checked with MoveIt's `/check_state_validity` before it is sent (collisions).
6. Ready pose (left button held): smooth joint-space move, about 11°/s average, at least 2 s;
   refused if the elbow, wrist or tip would come within `min_height` of the base height.
7. Health check every 2 s (`/controller_manager/list_hardware_components`, `list_controllers`):
   if the arm driver is not active (USB link lost) or `forward_position_controller` is not active
   (e.g. after switching back to planning mode), teleop pauses and says what to do; it resumes by
   itself when both are back.
8. While idle it re-reads the real arm pose every second, so a push never starts with a jump.

Why not MoveIt's `/compute_ik` per step (the first version): it is all-or-nothing. The puck
always leaks a little into other axes, so near any edge the whole step failed ("stuck"), and
the solver sometimes jumped to another arm configuration ("flip"). From the ready pose, straight-
line room before a hard stop went from 5.5–15.5 cm (exact IK) to 7–25 cm (this method), in the
offline model.

### 7.3 Parameters

Launch arguments: `start_mode` (joint), `linear_speed` (0.1), `angular_speed` (0.5), e.g.
`ros2 launch spacenav_arm_bridge spacemouse_teleop.launch.py start_mode:=tip linear_speed:=0.07`.
All node parameters (defaults; the launch file sets `rot_axis_map` to the calibrated value):

| Parameter | Default | Meaning |
|---|---|---|
| `start_mode` | joint | `joint` or `tip` |
| `linear_speed` | 0.1 | TIP mode tip speed at full deflection, m/s |
| `angular_speed` | 0.5 | TIP mode gripper turn speed at full deflection, rad/s (~29°/s); 0 = no rotation |
| `joint_speeds_deg` | [25, 25, 25, 40, 40, 40] | JOINT mode top speed per joint, °/s |
| `joint_inputs` | [ang_z, lin_y, lin_z, lin_x, ang_x, ang_y] | JOINT mode: puck input (arm frame, after the axis maps) driving each joint |
| `joint_signs` | [0, 0, 0, 0, 0, 0] | JOINT mode direction per joint; 0 = automatic, ±1 = override |
| `switch_margin` | 1.3 | JOINT mode: how much stronger another motion must be to take over |
| `deadband` | 0.1 | puck readings below this are ignored |
| `full_deflection` | 0.65 | puck reading that counts as full deflection |
| `expo` | 0.8 | response curve, 0 = linear, 1 = cubic |
| `mode_button` / `gripper_button` | 1 / 0 | `/spacenav/joy` indices (1 = left, 0 = right) |
| `hold_time` | 1.0 | s a button must be held for its hold action |
| `precision_scale` | 0.3 | speed factor in precision mode |
| `axis_map` | [1, 2, 3] | tip x, y, z ← puck axis number (1 = x, 2 = y, 3 = z); negative = reversed |
| `rot_axis_map` | [1, 2, 3] (launch: [−1, −2, 3]) | same for tilt/twist |
| `one_mode_at_a_time` | true | TIP mode: push or rotate, never both |
| `rotate_about` | wrist | TIP mode tilts: `wrist` (joints 4–6 only) or `tip` |
| `lead_time` | 0.0 | s to aim commands ahead along the motion (tested at 0.12: no benefit, section 7.5) |
| `max_joint_speed` | 0.6 | TIP mode cap per joint, rad/s |
| `damping` | 0.02 | damped-least-squares damping; higher = calmer near singularities |
| `rot_weight` | 0.5 | how firmly the tip orientation is held in TIP mode |
| `joint_limits_deg` | README limits | 12 values `[lo1, hi1, ..., lo6, hi6]`; J6 not stated in the README, ±180 |
| `limit_margin_deg` | 5.0 | safety margin inside each limit |
| `ready_pose_deg` | [0, −50, −60, 0, −40, 0] | target of the left-button hold |
| `home_speed` | 0.2 | rad/s average for the joint that moves most on the way to the ready pose |
| `min_height` | 0.05 | m; the ready-pose move is refused if the path comes lower |
| `resync_after` | 1.0 | s of no input before re-reading the real pose |
| `rate` | 30.0 | Hz |

### 7.4 Calibration

- **Push (`axis_map`) = `[1, 2, 3]`**, calibrated with `tools/axis_calib.py`: from the operator's
  seat, away = arm −x, left = arm −y, up = arm +z, and the puck reports the same.
- **Tilt/twist (`rot_axis_map`) = `[−1, −2, 3]`**, found on the arm: with `[1, 2, 3]` both tilts
  were reversed and twist was right; with `[−1, −2, 3]` both tilts turn the gripper the way the
  operator expects. In TIP mode a counter-clockwise twist (seen from above) spins the gripper
  counter-clockwise.
- JOINT mode needs no calibration of its own: it uses the two maps above.
- Rerun the calibration if the operator sits elsewhere or the arm or SpaceMouse is moved.

### 7.5 Wrist servo acceleration: the jitter fix

Phase 0 ([`gate_a_phase0_results.md`](gate_a_phase0_results.md)) showed joints 5–6 following a
smooth 5°/s target with ±1.6–1.7°/s speed ripple, against ±0.3–0.5°/s for joints 2–4, and
suggested their higher configured servo acceleration (100 vs 30 deg/s²) as the cause. Tested by
changing only that setting (`real_arm_tuned.launch.py wrist_acceleration:=...`); results in the
Phase 0 file, raw data in [`data/ramp_wrist_acceleration/`](data/ramp_wrist_acceleration):

| wrist acceleration (deg/s²) | speed ripple, joints 5–6 (deg/s) |
|---|---|
| 100 (arm repo default) | ±1.1–2.0 |
| 50 | ±0.7–1.2 |
| 30 (now the default of the tuned launch) | ±0.4–0.6, like joints 2–4 |

Aiming the command 0.12 s ahead (`lead_time`) did **not** reduce the ripple at acceleration 100,
so it is off. Joint 4 keeps the arm's 50 deg/s² (its ripple was already ±0.5°/s).

### 7.6 Open items

- **Gripper:** working and on the right button since 2026-09-23; not yet used together with the
  real arm (needs a USB hub). Open gripper items: [`gripper_cge_10_10.md`](gripper_cge_10_10.md),
  section 8.
- The ~0.3 s lag and the 0.3–3 s loop freezes remain (WSL2/usbipd; native Ubuntu should fix
  them, [`native_ubuntu_setup.md`](native_ubuntu_setup.md)).
- The J2/J3 coupled limit is not enforced.
- The RViz STOP button pause (the node listens to `/emergency_stop_button/feedback`) is not
  verified on the hardware; Ctrl-C in the teleop terminal is the tested way to stop.
- TIP mode: with the gripper model (default since 2026-09-23) `pro_arm_ee` is the model's gripper
  tip, so straight-line moves and rotations are about the gripper instead of the bare flange. Not
  yet tried on the real arm; joint directions and README limits are unchanged, and MoveIt accepts
  the ready pose, the all-zero pose and a −100° wrist bend with the gripper (checked in simulation).

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
| `arm_characterize.py all` | Gate A Phase 0: standstill noise, loop timing, per-joint step response, backlash, speed/acceleration, moving-target ripple; writes CSV + summary | forward controller active, teleop not running, ready pose |
| `arm_characterize.py play SECONDS` | live joint readings while you push the arm by hand | same |
| `axis_calib.py` | moves the tip ±3 cm along each arm axis (and back), asks which way it went, records three puck pushes, prints `axis_map` | planning mode, `spacenav_node`, arm in the working pose |
| `gripper_test.py selftest\|status\|test\|pos N\|force N\|speed N\|init\|watch S` | the CGE-10-10 over RS-485 without ROS (details in [`gripper_cge_10_10.md`](gripper_cge_10_10.md)); CSV to `~/gripper_logs/` | the USB-RS485 adapter attached; not while `gripper_node` runs |

`arm_timer.py` and `fwd_test.py` refuse moves over 0.6 rad per command; `axis_calib.py` refuses
test moves that swing any joint more than 0.35 rad. Keep the arm's reach clear when running them.
