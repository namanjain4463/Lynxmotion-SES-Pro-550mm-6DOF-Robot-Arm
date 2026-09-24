# DH-Robotics CGE-10-10 gripper: driver, grasp/slip detection, measured behaviour (2026-09-23)

The 3-finger centric gripper of the Lynxmotion SES-Pro kit, driven for the first time from ROS.
Everything below ran on the real gripper unless marked otherwise.

## 1. What was missing and how it is solved

- The arm repo (and Lynxmotion's official `SES-P-ROS2-Arms`, `PRO-ROS2-Arms`, `LSS-P-ROS2-Hardware`)
  has **no gripper driver**: `pro_arm.ros2_control` labels the gripper block "simulation only" and
  uses `fake_components/GenericSystem` even in real mode. The arm repo README's
  `ign_ros2_control` → `gz_ros2_control` rename (its item 3) is a Gazebo warning fix, already
  applied, and unrelated to the real gripper.
- DH-Robotics' own driver ([dh_gripper_ros](https://github.com/DH-Robotics/dh_gripper_ros)) is ROS 1
  and does not list the CGE (it covers PGE/PGC/CGC/AG-95 with one register set).
- The gripper is a **separate Modbus RTU device on RS-485**, not on the arm's LSS-PRO bus. It gets
  its own USB-RS485 adapter and its own serial port. This folder adds a plain-pyserial driver
  (`spacenav_arm_bridge/dh_gripper.py`), a test tool (`tools/gripper_test.py`) and a ROS 2 node
  (`spacenav_arm_bridge/gripper_node.py`). Nothing is WSL-specific (for the Jetson later).

## 2. Hardware and wiring

| Item | Value |
|---|---|
| Adapter | CH340 USB-serial with RS-485, VID:PID **`1a86:7523`** → `/dev/ttyUSB0` (kernel driver `ch341`) |
| Power | 24 V DC (±10 %) |
| Bus settings | slave ID **1**, **115200** baud, 8N1 (factory defaults, confirmed by the first read) |

8-wire gripper cable (DH CGE-10 short manual, table 1.3):

| Wire | Signal | Connect to |
|---|---|---|
| Red | +24 V | supply + |
| Grey/Pink | GND | supply − (and the adapter's GND, if it has one) |
| Black | RS-485 A | adapter A |
| Blue | RS-485 B | adapter B |
| White / Brown | Input 1 / 2 | not used |
| Yellow / Green | Output 1 / 2 | not used |

On WSL2 the adapter needs usbipd like the arm and the SpaceMouse (PowerShell, admin):
```powershell
usbipd bind --hardware-id 1a86:7523
usbipd attach --wsl --hardware-id 1a86:7523 --auto-attach
```
**The test laptop has only two USB ports**, so arm + SpaceMouse + gripper together need a USB hub
(a powered one, given the arm's history of link drops, section 5.2 of the README). Until then the
gripper was tested with the SpaceMouse and the **simulated** arm (`sim_arm_tuned.launch.py`).

A blinking red light on the gripper after power-on means "not initialised" (DH indicator
convention; blue = initialised, green = object caught, blinking green = dropped).

## 3. Register map (only these are used)

From the DH CGE-10 short manual, section 2.3 ([ManualsLib copy](https://www.manualslib.com/manual/2822598/Dh-Robotics-Cge-10.html)).
Cross-checked three ways: all 13 example frames in the manual pass the Modbus CRC; DH-Robotics'
`dh_gripper_driver` (`DH_Modbus_Gripper`) and the independent `pyDHgripper` (PGE) use the same
addresses; and the real gripper answered every read and echoed every write. One automated summary
of the manual gave position/speed as 0x0102/0x0103; the manual's own table, its CRC-valid example
frames and both drivers say **0x0103/0x0104**.

| Register | Meaning | Values |
|---|---|---|
| `0x0100` | initialise | write 1 = initialise, 0xA5 = full initialise |
| `0x0101` | force (a current limit, **not a measured force**) | 20–100 % |
| `0x0103` | target position (the fingers move at once) | 0–1000 ‰ |
| `0x0104` | speed | 1–100 % |
| `0x0200` | initialised? | 0 no, 1 yes |
| `0x0201` | grip state | 0 moving, 1 at target, 2 object caught, 3 object dropped; **65535 before initialisation** (measured) |
| `0x0202` | actual position | 0–1000 ‰ |

The tool and the node never write the `0x03xx` configuration registers (slave ID, baud rate,
save), so they cannot change how the gripper communicates.

## 4. Measured behaviour

**Direction and timing**
- **1000 ‰ = open, 0 = closed** (operator-confirmed). Standard initialisation took 1.4 s and left
  the fingers at 1000.
- Full stroke ≈ 0.7 s at speed 30 %, ≈ 0.6 s at speed 50 %; with nothing in the way the fingers
  reach 1600–3100 ‰/s at speed 50 %.
- Ends are exact (0 and 1000). Mid-stroke targets stop 2–3 % of stroke off: 700 → 682,
  500 → 523 (single readings at the moment the gripper reported "at target").

**Aperture (finger mounting as delivered)**
- The Lynxmotion kit mounts the fingers in one of three positions, drawn with 20 / 40 / 60 mm
  circles ([kit page](https://wiki.lynxmotion.com/info/wiki/lynxmotion/view/ses-pro/ses-pro-parts/ses-p-dhr-cge-10-10-kt/)).
  With a 10 mm stroke per jaw (DH spec) each position covers a band about 20 mm wide in diameter.
  The installed fingers point outward past the housing, like the drawing's 60 mm position.
- A **51 mm** cylinder was caught at **917–987 ‰** (9 grasps, 2026-09-23 21:05–21:11 and
  22:36–22:38), i.e. only 1–8 % of stroke after fully open. So fully open is only about 1 mm
  wider than 51 mm: **open ≈ 52 mm**.
- **Closed ≈ 32 mm is an estimate** (open minus the 20 mm stroke), not measured. The node's size
  estimate uses this linear map (51.6 mm at 1000, 31.6 mm at 0; parameters `diameter_open_mm`,
  `diameter_closed_mm`) and gave 49.9–51.3 mm for the 51 mm cylinder. That is consistency with
  the object it was anchored on, not validation: **a second object of known diameter (e.g. a
  printed Ø 40 mm band, expected near 420 ‰) is needed.**
- Objects under ~32 mm are not caught at all in this mounting (a pen closed fully without contact).
- This explains the old simulator discrepancy (see `docs/lynxmotion_embodiment_results.md` in the
  private Thesis repo): the
  MuJoCo mesh measurement (~3–18 mm) matches the *smallest* mounting band, not this one; the
  model's `finger` argument (20/40/60) must match the real mounting before grasp simulations.
  Measured from the arm repo's finger meshes (inner fingertip circle, last 10 mm of the fingers,
  `joint_7` 0 → 0.01 m): `finger 20` ≈ 11–24 → 3–7 mm; **`finger 40` ≈ 35–46 → 16–26 mm**, the
  closest to the real ~52 → ~32 mm; `finger 60` is asymmetric (fingertip radii 12 / 32 / 20 mm),
  so that model variant looks broken. None matches the real fingers exactly.

**Grasp and loss signatures** (51 mm cylinder and other objects, force 20 %, removed by hand)
- A normal hold creeps **0–31 ‰** in the first 1.5 s (seating), then stays flat.
- An object sliding **while still touched** lets the fingers creep at **10–130 ‰/s**.
- Once it is **gone**, the fingers close freely at **1600–3100 ‰/s**, a >10× separation.
- The grip state lags: it stays "object caught" while the object slides out and changes only
  after the fingers have closed on air; right after a new command it can show the previous
  result for ~0.4 s. **The finger position is the slip sensor; the state is not.**

**Communication** (ROS node, 233 s session): 10.0 Hz polling, longest gap 182 ms, no Modbus
errors or reconnects (node log and state log).

## 5. Test tool: `tools/gripper_test.py` (no ROS)

```bash
python3 ~/gripper_test.py selftest      # no hardware: reproduces the manual's frames byte for byte
python3 ~/gripper_test.py status -v     # read-only; -v prints every Modbus frame
python3 ~/gripper_test.py test          # guided: init, moves, open/closed direction, optional grip
python3 ~/gripper_test.py pos 1000      # also: init [--full], force N, speed N, watch SECONDS
```
Options `--port --baud --id`. Every run writes a CSV to `~/gripper_logs/`. The guided test uses
force 20 % and speed 30 %.

## 6. ROS 2 node: `gripper_node`

Started by the teleop launch with `gripper:=true` (also `gripper_port`, `gripper_force`,
`gripper_log_dir`), or alone with `ros2 run spacenav_arm_bridge gripper_node`. The right
SpaceMouse button (short press) calls `/gripper/toggle`.

| Interface | Type | Content |
|---|---|---|
| `/gripper/open`, `/gripper/close`, `/gripper/toggle` | `std_srvs/Trigger` | toggle opens if closed or holding, else closes |
| `/gripper/set_position`, `/gripper/set_force` | `std_msgs/Int32` | 0–1000 ‰; 20–100 % |
| `/gripper/position`, `/gripper/grip_state` | `std_msgs/Int32` | as the registers |
| `/gripper/object_caught` | `std_msgs/Bool` | grip state = 2 |
| `/gripper/diameter_mm` | `std_msgs/Float32` | size estimate while holding, NaN otherwise (provisional map) |
| `/gripper/event` | `std_msgs/String` (JSON) | grasp events, below |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | connection, poll rate, read time, Modbus errors, reconnects, grasp phase (1 Hz) |

Parameters (besides `model_action`, `model_open_q`, `model_closed_q` above): `port` (/dev/ttyUSB0), `baud` (115200), `slave_id` (1), `force_pct` (20), `speed_pct`
(50), `rate` (10 Hz), `seat_time` (1.5 s), `slip_permille` (20), `contact_loss_speed`
(500 ‰/s), `diameter_open_mm` / `diameter_closed_mm` (51.6 / 31.6, provisional), `arm_moving_deg`
(0.5), `log_dir` ("" = no logs). With `log_dir` it writes three CSVs per session:
`*_gripper.csv` (every state sample with the commanded position and force),
`*_gripper_events.csv` and `*_gripper_diagnostics.csv` (1 Hz).

On start it initialises the gripper if needed (the fingers move). On shutdown it only closes the
port: the gripper keeps its position, so a held object is not dropped. If the adapter disappears
it reconnects by itself.

### RViz / MoveIt model

`real_arm_tuned.launch.py` and `sim_arm_tuned.launch.py` put the CGE-10-10 into the model by default
(`gripper:=cge_1010 finger:=40`; `gripper:=none` for the old model). The arm repo's launch then also
starts `gripper_action_controller` on the model's `joint_7`, which is **simulated even with the
real arm**. `gripper_node` sends the real finger position to it (`model_action`, default
`/gripper_action_controller/gripper_cmd`; `joint_7` = 0 m open … 0.01 m closed; mirrored when the
position changes by 10 ‰, at most every 0.1 s), so **the fingers in RViz follow the real
gripper**. It only mirrors, it does not add accuracy: the model's fingers are not the installed
ones (above). Checked in simulation with a simulated gripper: a catch at 650 ‰ gave
`joint_7` = 0.0036 m, opening gave 0; the arm-motion check ignores `joint_7`.

### Grasp events (`grasp_monitor.py`, pure Python, replayable on logs)

| Event | When | Recorded |
|---|---|---|
| CAUGHT | fingers stop on an object during a close | position, diameter estimate, close time |
| SEATED | 1.5 s after the catch, if no slip yet | seating creep |
| SLIP | in-contact creep beyond seating: at 20 ‰, then each time it doubles | creep, time slipping, arm moving? |
| LOST | object gone without an open command; detected by the fingers speeding up past 500 ‰/s (or the state / closed end) | GRADUAL (slid in contact ≥ 0.2 s first) or SUDDEN; in-contact slip time and amount; arm moving → "dropped during arm motion", arm still → "removed at rest" |
| RELEASED | opened on purpose while holding | hold time, creep, hold horizons survived (0.12 / 0.5 / 2 s, the simulator benchmark's) |
| MISSED | a close ended fully shut on nothing | position |

Every event carries the **commanded** force (the squeeze setting), which a camera cannot see.

How it was checked: replaying the two real teleop sessions of 2026-09-23 reproduces every catch,
release, removal and empty close; the two sessions' removals come out as SUDDEN (in one go) and
GRADUAL (0.3–1.5 s of in-contact slip). The arm-motion classification and the diagnostics CSV
were tested with a simulated gripper and simulated `/joint_states`, not yet during real arm motion.

## 7. Limits

- The fingers sense only motion toward the centre. An object that **tips or rotates** in the
  grip without the fingers closing further (the simulator's TIP failure) is invisible to the
  gripper; that needs the camera. The gripper covers the DROP side.
- One coupled position for three fingers; no per-finger contact or force.
- Force % is a current limit, not newtons; no current/force readback register is used (none is in
  the register table).
- DH recommends ≤ 0.1 kg workpieces; 3–10 N per jaw.
- "Removed at rest" vs "dropped during motion" is inferred from arm motion, not sensed.
- Speeds and thresholds (`contact_loss_speed` 500 ‰/s) were measured at speed 50 %; recheck them if
  the speed setting changes a lot.

## 8. Not done yet

- Arm, SpaceMouse and gripper together on the real arm (needs a USB hub); a real pick-and-place.
- A second known diameter to confirm the size map; the Gate A protocol's gripper
  characterisation (10 cycles at two speeds, five aperture levels in both directions measured
  physically, three diameters × 10 closes).
- Force comparison (does seating creep grow with force?), and slip vs force with a ballasted object.
- Object pose from the camera during holds (tipping, and slip that the fingers cannot see).
