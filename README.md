# openflex_vr_bridge

English | [中文](./README-CN.md)

---

ROS 2 node that receives UDP datagrams from Pico or quest VR controllers and body trackers, and publishes them as ROS 2 topics.

## Overview

This C++ node acts as a bridge between the Pico VR headset/controllers and the ROS 2 ecosystem. It listens on a configurable UDP port for pose data sent by the OpenFlex Pico APK running on the VR headset, then publishes controller poses, trigger/grip values, joystick axes, button states, body tracker poses, and optionally TF transforms.

## Published Topics

### Controller Topics (per hand: left/right)

| Topic | Type | Description |
|-------|------|-------------|
| `/pico_{left,right}_controller/pose` | `geometry_msgs/PoseStamped` | Controller 6-DOF pose (position + quaternion) |
| `/pico_{left,right}_controller/trigger` | `std_msgs/Float32` | Index trigger value (0-1) |
| `/pico_{left,right}_controller/grip` | `std_msgs/Float32` | Grip trigger value (0-1) |
| `/pico_{left,right}_controller/joystick_x` | `std_msgs/Float32` | Joystick X axis (-1 to 1) |
| `/pico_{left,right}_controller/joystick_y` | `std_msgs/Float32` | Joystick Y axis (-1 to 1) |
| `/pico_{left,right}_controller/joystick_click` | `std_msgs/Bool` | Joystick press state |
| `/pico_{left,right}_controller/rate` | `std_msgs/Float32` | Speed multiplier (0.1 or 1.0) |

### Button Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/pico_right_controller/button_a` | `std_msgs/Bool` | A button (right controller) |
| `/pico_right_controller/button_b` | `std_msgs/Bool` | B button (right controller) |
| `/pico_left_controller/button_x` | `std_msgs/Bool` | X button (left controller) |
| `/pico_left_controller/button_y` | `std_msgs/Bool` | Y button (left controller) |

### Body Tracker Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/pico_tracker/waist/pose` | `geometry_msgs/PoseStamped` | Waist tracker pose |
| `/pico_tracker/left_foot/pose` | `geometry_msgs/PoseStamped` | Left foot tracker pose |
| `/pico_tracker/right_foot/pose` | `geometry_msgs/PoseStamped` | Right foot tracker pose |
| `/pico_head/pose` | `geometry_msgs/PoseStamped` | Head/HMD pose |

### Chassis Speed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/pico_chassis/max_linear_speed` | `std_msgs/Float32` | VR-configured max linear speed |
| `/pico_chassis/max_angular_speed` | `std_msgs/Float32` | VR-configured max angular speed |

### TF (optional, disabled by default)

- `pico_hmd -> pico_left_controller`
- `pico_hmd -> pico_right_controller`

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `listen_address` | `0.0.0.0` | UDP listen address |
| `listen_port` | `5100` | UDP listen port |
| `frame_id` | `pico_hmd` | TF parent frame |
| `publish_tf` | `false` | Whether to publish TF transforms |
| `left_pose_topic` | `/pico_left_controller/pose` | Left controller pose topic |
| `right_pose_topic` | `/pico_right_controller/pose` | Right controller pose topic |

## UDP Protocol

The node accepts text-based datagrams in several formats:

- **HAND** `L/R pos_x pos_y pos_z qx qy qz qw trigger grip [buttons] [joystick_x joystick_y] [rate] [timestamp_ns]`
- **BTN** `L/R button_id pressed [timestamp_ns]`
- **JOY** `L/R joystick_x joystick_y [timestamp_ns]`
- **TRIG** `L/R trigger_value [timestamp_ns]`
- **RATE** `rate_value [timestamp_ns]`
- **CFG** `LIN/ANG speed_value`
- **WAIST/LEFT_FOOT/RIGHT_FOOT/HEAD** `pos_x pos_y pos_z qx qy qz qw [timestamp_ns]`
- Legacy format: `L/R pos_x pos_y pos_z qx qy qz qw trigger grip ...`

## Usage

```bash
# Build

---
cd ~/openflex_all/openflex_ws
colcon build --packages-select openflex_vr_bridge

# Run standalone

---
ros2 run openflex_vr_bridge pico_pose_bridge_node --ros-args -p listen_port:=5100

# Run with parameters

---
ros2 run openflex_vr_bridge pico_pose_bridge_node --ros-args \
  -p listen_port:=5100 \
  -p publish_tf:=true \
  -p frame_id:=pico_hmd
```

## APK

The VR APK files are distributed outside this ROS package under `~/openflex_all/openflex_vr_apk/apk/`: `pico/OpenFlex.apk` is the Pico Android app, and `quest/openarmx-vr-quest.apk` is the Quest build. The app streams controller/tracker data to this bridge node via UDP. Install the matching APK on the VR device and ensure network connectivity.

## Prerequisites

- ROS 2 (Humble or later)
- Pico VR headset running the OpenFlex APK on the same network
- UDP port 5100 (default) open and accessible

## License

This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License (CC BY-NC-SA 4.0).

Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)

For more details, see the [LICENSE](LICENSE) file or visit: http://creativecommons.org/licenses/by-nc-sa/4.0/

## Acknowledgments

This package is part of the OpenArmX robotic platform ecosystem, developed for research and industrial applications in collaborative robotics.

---

## 📞 Contact Us

### Chengdu Changshu Robot Co., Ltd.

| Contact           | Information                                                                                                  |
| ----------------- | ------------------------------------------------------------------------------------------------------------ |
| 📧 Email          | [openarmrobot@gmail.com](mailto:openarmrobot@gmail.com)                                                      |
| 📱 Phone / WeChat | +86-17746530375                                                                                              |
| 🌐 Website        | [https://openarmx.com/](https://openarmx.com/)                                                               |
| 🌐 Documentation  | [http://docs.openarmx.com/](http://docs.openarmx.com/)                                                               |
| 📍 Address        | Huacheng Machinery Plant, No.11 Xinye 8th Street, West Area, Tianjin Economic-Technological Development Area |
| 👤 Contact Person | Mr. Wang                                                                                                     |
