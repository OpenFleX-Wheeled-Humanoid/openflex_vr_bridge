# openflex_vr_bridge

[English](./README.md) | 中文

---

接收 Pico 和 quest VR 手柄和体感追踪器 UDP 数据报并发布为 ROS 2 话题的节点。

## 概述

本 C++ 节点充当 Pico VR 头显/手柄与 ROS 2 生态系统之间的桥梁。它在可配置的 UDP 端口上监听由 VR 头显上运行的 OpenFlex Pico APK 发送的姿态数据，然后发布手柄位姿、扳机/握把值、摇杆轴、按键状态、体感追踪器位姿，以及可选的 TF 变换。

## 发布话题

### 手柄话题（每只手：left/right）

| 话题 | 类型 | 描述 |
|------|------|------|
| `/pico_{left,right}_controller/pose` | `geometry_msgs/PoseStamped` | 手柄 6-DOF 位姿（位置 + 四元数） |
| `/pico_{left,right}_controller/trigger` | `std_msgs/Float32` | 食指扳机值（0-1） |
| `/pico_{left,right}_controller/grip` | `std_msgs/Float32` | 握把扳机值（0-1） |
| `/pico_{left,right}_controller/joystick_x` | `std_msgs/Float32` | 摇杆 X 轴（-1 到 1） |
| `/pico_{left,right}_controller/joystick_y` | `std_msgs/Float32` | 摇杆 Y 轴（-1 到 1） |
| `/pico_{left,right}_controller/joystick_click` | `std_msgs/Bool` | 摇杆按下状态 |
| `/pico_{left,right}_controller/rate` | `std_msgs/Float32` | 速度倍率（0.1 或 1.0） |

### 按键话题

| 话题 | 类型 | 描述 |
|------|------|------|
| `/pico_right_controller/button_a` | `std_msgs/Bool` | A 键（右手柄） |
| `/pico_right_controller/button_b` | `std_msgs/Bool` | B 键（右手柄） |
| `/pico_left_controller/button_x` | `std_msgs/Bool` | X 键（左手柄） |
| `/pico_left_controller/button_y` | `std_msgs/Bool` | Y 键（左手柄） |

### 体感追踪器话题

| 话题 | 类型 | 描述 |
|------|------|------|
| `/pico_tracker/waist/pose` | `geometry_msgs/PoseStamped` | 腰部追踪器位姿 |
| `/pico_tracker/left_foot/pose` | `geometry_msgs/PoseStamped` | 左脚追踪器位姿 |
| `/pico_tracker/right_foot/pose` | `geometry_msgs/PoseStamped` | 右脚追踪器位姿 |
| `/pico_head/pose` | `geometry_msgs/PoseStamped` | 头显位姿 |

### 底盘速度话题

| 话题 | 类型 | 描述 |
|------|------|------|
| `/pico_chassis/max_linear_speed` | `std_msgs/Float32` | VR 配置的最大线速度 |
| `/pico_chassis/max_angular_speed` | `std_msgs/Float32` | VR 配置的最大角速度 |

### TF（可选，默认禁用）

- `pico_hmd -> pico_left_controller`
- `pico_hmd -> pico_right_controller`

## 参数

| 参数 | 默认值 | 描述 |
|------|--------|------|
| `listen_address` | `0.0.0.0` | UDP 监听地址 |
| `listen_port` | `5100` | UDP 监听端口 |
| `frame_id` | `pico_hmd` | TF 父坐标系 |
| `publish_tf` | `false` | 是否发布 TF 变换 |
| `left_pose_topic` | `/pico_left_controller/pose` | 左手柄位姿话题 |
| `right_pose_topic` | `/pico_right_controller/pose` | 右手柄位姿话题 |

## UDP 协议

节点接受以下文本格式的数据报：

- **HAND** `L/R pos_x pos_y pos_z qx qy qz qw trigger grip [buttons] [joystick_x joystick_y] [rate] [timestamp_ns]`
- **BTN** `L/R button_id pressed [timestamp_ns]`
- **JOY** `L/R joystick_x joystick_y [timestamp_ns]`
- **TRIG** `L/R trigger_value [timestamp_ns]`
- **RATE** `rate_value [timestamp_ns]`
- **CFG** `LIN/ANG speed_value`
- **WAIST/LEFT_FOOT/RIGHT_FOOT/HEAD** `pos_x pos_y pos_z qx qy qz qw [timestamp_ns]`
- 旧格式：`L/R pos_x pos_y pos_z qx qy qz qw trigger grip ...`

## 使用方法

```bash
# 编译

---
cd ~/openflex_all/openflex_ws
colcon build --packages-select openflex_vr_bridge

# 独立运行

---
ros2 run openflex_vr_bridge pico_pose_bridge_node

# 带参数运行

---
ros2 run openflex_vr_bridge pico_pose_bridge_node --ros-args \
  -p listen_port:=5100 \
  -p publish_tf:=true \
  -p frame_id:=pico_hmd
```

## APK

VR APK 文件放在 ROS 包外部的 `~/openflex_all/openflex_vr_apk/apk/` 下：`pico/OpenFlex.apk` 是运行在 Pico VR 头显上的 Android 应用；`quest/openarmx-vr-quest.apk` 是 Quest 版本。应用通过 UDP 向本桥接节点发送手柄/追踪器数据。请将对应 APK 安装到 VR 设备上并确保网络连通。

## 前置条件

- ROS 2（Humble 或更高版本）
- Pico VR 头显在同一网络运行 OpenFlex APK
- UDP 端口 5100（默认）开放且可访问

## 许可证

本作品采用知识共享 署名-非商业性使用-相同方式共享 4.0 国际许可协议 (CC BY-NC-SA 4.0) 进行许可。

版权所有 (c) 2026 成都长数机器人有限公司 (Chengdu Changshu Robot Co., Ltd.)

详情请参阅 [LICENSE_CN.md](LICENSE) 文件或访问：http://creativecommons.org/licenses/by-nc-sa/4.0/

## 致谢

本包是 OpenArmX 机器人平台生态系统的一部分，专为协作机器人领域的研究和工业应用而开发。

---

## 📞 联系我们

### 成都长数机器人有限公司
**Chengdu Changshu Robotics Co., Ltd.**

| 联系方式 | 信息 |
|---------|------|
| 📧 邮箱 | openarmrobot@gmail.com |
| 📱 电话/微信 | +86-17746530375 |
| 🌐 官网 | <https://openarmx.com/> |
| 🌐 文档 | <http://docs.openarmx.com/> |
| 📍 地址 | 天津经济技术开发区西区新业八街11号华诚机械厂 |
| 👤 联系人 | 王先生 |
