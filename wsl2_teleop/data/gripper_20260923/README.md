# Gripper logs, 2026-09-23 (DH CGE-10-10, real gripper)

Summary and conclusions: [`../../gripper_cge_10_10.md`](../../gripper_cge_10_10.md).

| File | What |
|---|---|
| `gripper_test/*_test.csv` | `gripper_test.py test` (20:59): initialise, moves 700/1000/0/500, grip test that missed (object outside the jaws) |
| `gripper_test/*_pos.csv` | `gripper_test.py pos` runs (21:03–22:02): open/close cycles; the 51 mm cylinder caught at 952–987 ‰ (21:05–21:11); two catches at 157/171 ‰ (21:04) of an object not recorded |
| `20260923_222341_gripper.csv` | ROS `gripper_node` state log (5 Hz, before grasp detection existed), right-button toggling with the simulated arm; catches of several objects, two removals by hand |
| `20260923_223604_gripper.csv` + `_events.csv` | `gripper_node` with grasp detection (10 Hz), simulated arm, 51 mm cylinder: 2 empty closes, 1 normal hold, 3 removals by hand. The events file was written by the first version of the loss rule (all removals GRADUAL); replaying this state log through the current `grasp_monitor.py` gives SUDDEN / GRADUAL 0.3 s / GRADUAL 1.5 s. The state log ends ~5 s before the events (unflushed buffer; fixed). The last catch (121 ‰, 767 s) was a different object. |

Columns of the node logs: `t_s` (s since node start), `commanded_position` (‰, last target sent),
`force_pct` (commanded), `initialised`, `grip_state` (0 moving, 1 at target, 2 caught, 3 dropped),
`position` (‰, 1000 = open). All forces are the commanded setting (20 %), not measured.
