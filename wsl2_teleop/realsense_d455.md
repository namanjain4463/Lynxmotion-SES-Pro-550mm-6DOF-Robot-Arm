# Intel RealSense D455: install and first test (2026-09-25)

Tested on the laptop (Windows 11 + WSL2 Ubuntu 22.04, ROS 2 Humble, usbipd). The lab computer
(native Ubuntu 22.04) has not been tested yet; section 4 lists what to check there.

## 1. Camera and packages

| Item | Value |
|---|---|
| Camera | D455, VID:PID `8086:0b5c`, serial 313522302002, firmware 5.17.3.10 |
| Packages | `ros-humble-librealsense2` 2.58.4 (library + `rs-*` tools), `ros-humble-realsense2-camera` 4.58.4, `ros-humble-realsense2-description` (all from the ROS apt repository, no source build) |
| Permissions | Intel's udev rules `/etc/udev/rules.d/99-realsense-libusb.rules`; user in `video` (and `plugdev`) |

Install (Ubuntu or WSL terminal):
```bash
sudo apt update
sudo apt install -y ros-humble-librealsense2 ros-humble-realsense2-camera ros-humble-realsense2-description ros-humble-rqt-image-view
sudo curl -fsSL https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules -o /etc/udev/rules.d/99-realsense-libusb.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## 2. Connection

- **USB 3 is needed.** The first cable/port connected the camera at USB 2 (480 Mbit/s); with the
  D455's USB 3 cable in a USB 3 port it ran at 5000 Mbit/s ("Device USB type: 3.2"). Check with
  `lsusb -t` (the `uvcvideo` lines must say `5000M`). RealSense USB-C plugs can be sensitive to
  orientation and seating.
- **WSL2 only:** share it with `usbipd bind --force --hardware-id 8086:0b5c`, then
  `usbipd attach --wsl --hardware-id 8086:0b5c --auto-attach` (PowerShell, admin). With a plain
  `bind`, Windows closed the connection about 0.3 s after every attach (kernel log:
  `vhci_hcd: connection closed`). `--force` keeps Windows from using the camera until
  `usbipd unbind --hardware-id 8086:0b5c`. After changing port, stop the attach loop and bind/attach
  again (the loop follows one bus ID).

## 3. Results under WSL2 (usbipd)

**Detection works** (`rs-enumerate-devices -s`).

**The IMU does not work under WSL2:** librealsense's Linux backend reads it through the kernel's
HID sensor hub, which the WSL kernel does not include (`CONFIG_HID_SENSOR_HUB` not set). Messages:
`No HID info provided, IMU is disabled`, `HID Motion Sensor Failure (continuing as partial device)`
and, in the ROS node, `Motion Module force pause ... Hardware Error`. Depth and colour are
unaffected.

**Large frames are lost over usbipd.** What limits it is the size of each frame, not the frame
rate (`rs-data-collect`, 8–10 s per setting, frame numbers from the camera):

| Setting (alone unless stated) | Delivered |
|---|---|
| depth 480×270 @ 15 / 30 | 14.9 / 30.0 fps, 0–1 % lost |
| depth 640×360 @ 15 / 30 | 12.4 / 25.7 fps, 14–17 % lost |
| depth 640×480 @ 5 / 15 | 4.7 / 9.1 fps, 5 / 39 % lost |
| depth 848×480 @ 5 / 15 | 4.3 / 1.1 fps, 13 / 92 % lost |
| colour 640×480 @ 15 / 30 | 14.6 / 29.2 fps, 2–3 % lost |
| depth 480×270 @ 15 + colour 640×480 @ 15 | 14.0 + 14.7 fps |

**ROS 2 node** (`realsense2_camera`, 15 s per setting, camera ~1.6 m from the scene; rates from a
subscriber counting received images, not `ros2 topic hz`, which under-reports large image topics):

| depth / colour profile | depth | colour | aligned depth |
|---|---|---|---|
| 480x270x30 / 640x480x30 | 30.0 Hz | 11.0 Hz (gaps to 0.8 s) | 8.0 Hz |
| 480x270x15 / 640x480x15 | 15.1 Hz | 5.4 Hz | 3.1 Hz |
| **480x270x30 / 424x240x30** | **29.9 Hz** | **29.4 Hz** | **29.8 Hz** |

Valid depth: 88–89 % of pixels (85 % after alignment), median distance 1.59 m. So under WSL2 use:
```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=480x270x30 rgb_camera.color_profile:=424x240x30 align_depth.enable:=true enable_gyro:=false enable_accel:=false
```
and view it with `ros2 run rqt_image_view rqt_image_view` in a second terminal. Topics:
`/camera/camera/color/image_raw`, `/camera/camera/depth/image_rect_raw`,
`/camera/camera/aligned_depth_to_color/image_raw` (+ `camera_info`, `metadata`,
`/camera/camera/extrinsics/depth_to_color`).

Other notes:
- Only one program can hold the camera. A leftover `realsense2_camera_node` makes the next one fail
  with `xioctl(VIDIOC_S_FMT) failed, errno=16 ... Device or resource busy`.
- `No valid configuration file found at ~/.realsense-config.json` is harmless.

## 4. Native Ubuntu 22.04 (lab computer / Jetson): what to check

Expected but **not yet verified**: no usbipd, so full-size frames should arrive, and the stock
Ubuntu kernel has the HID sensor hub, so the IMU should work. First lab-PC result (grip pilot,
2026-09-25): **colour-only 1280×720 at 30 Hz, USB 3.2, 600 of 600 frames in 20 s**
(`docs/lynxmotion_grip_pilot_protocol.md` in the private Thesis repo); depth,
colour together at full size and the IMU are still to be checked. Check:
1. `lsusb -t` shows `5000M`; `rs-enumerate-devices -s` finds the camera.
2. Full resolution plus IMU with `rs-data-collect -c c.txt -f out.csv -t 10`, where `c.txt` is
   ```
   DEPTH,848,480,30,Z16,0
   COLOR,1280,720,30,RGB8,0
   ACCEL,1,1,100,MOTION_XYZ32F,0
   GYRO,1,1,200,MOTION_XYZ32F,0
   ```
   (no "IMU is disabled" warning; frame counts close to 10 s × fps).
3. The ROS node at full settings:
   `ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=848x480x30 rgb_camera.color_profile:=1280x720x30 align_depth.enable:=true enable_gyro:=true enable_accel:=true unite_imu_method:=2`,
   then `ros2 topic hz` on the depth, colour, aligned and `/camera/camera/imu` topics.
4. If frames still drop, step down (848x480x15, then 640x480x30) and record which settings hold.

For the thesis measurements (protocol review, 2026-09-22): the D455's minimum depth is about
0.52 m at full resolution (lower resolutions reach closer); validate the chosen profile at the
real working distance, and keep raw depth with invalid-pixel masks (no hole filling).
