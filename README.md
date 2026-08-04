# PX4 Offboard Control

这是一个面向初学者的 ROS 1 + MAVROS + PX4 Offboard 控制示例，包含两种控制方式：

1. 发布目标位置，由 PX4 内部控制器完成位置、速度和姿态控制。
2. 自己实现位置外环和速度 PD 内环，计算目标姿态与推力后发送给 PX4。

代码中保留了较详细的中文注释，适合配合 Gazebo 仿真理解 Offboard 控制流程。

> 仅用于仿真和学习。PD 参数、悬停油门和安全限制未针对真实无人机标定，请勿直接用于真机。

## 系统结构

```text
ROS 控制节点
    |
    | ROS topic / service
    v
MAVROS
    |
    | MAVLink over UDP
    v
PX4 SITL <----> Gazebo
```

- **Gazebo**：模拟无人机、重力、传感器和物理环境。
- **PX4 SITL**：在电脑上运行的 PX4 飞控固件。
- **MAVROS**：在 ROS 消息和 MAVLink 消息之间转换。
- **Offboard 节点**：持续向 PX4 发布外部控制目标。

## 功能

### 位置控制节点

源码：`src/offboard_control/src/offboard_position.cpp`

发布：

```text
/mavros/setpoint_position/local
```

节点只发送目标位置，PX4 内部的位置控制器负责产生速度、姿态和电机控制量。这种方式简单、稳定，建议初学者先运行它。

### 位置-速度 PD 节点

源码：`src/offboard_control/src/offboard_pd_controller.cpp`

控制流程：

```text
目标位置
  -> 位置误差
  -> 位置 P 控制器
  -> 期望速度
  -> 速度误差
  -> 速度 PD 控制器
  -> 期望加速度
  -> 姿态四元数和推力
  -> PX4 姿态控制器
```

发布：

```text
/mavros/setpoint_raw/attitude
```

这个节点绕过 PX4 的位置环和速度环，但仍使用 PX4 内部姿态控制器。它不等于直接控制电机。

## 目录结构

```text
px4_offboard_ws/
├── README.md
├── .gitignore
└── src/
    └── offboard_control/
        ├── CMakeLists.txt
        ├── package.xml
        ├── config/
        │   └── pd_params.yaml
        ├── launch/
        │   ├── offboard_position.launch
        │   └── offboard_pd.launch
        └── src/
            ├── offboard_position.cpp
            └── offboard_pd_controller.cpp
```

## 推荐环境

本项目是 **ROS 1 catkin** 包，推荐在 Ubuntu 环境中运行：

- Ubuntu 20.04
- ROS Noetic
- MAVROS
- PX4-Autopilot SITL
- 与所选 PX4 版本匹配的 Gazebo / Gazebo Classic

当前项目目录虽然可以保存在 macOS 上，但 ROS Noetic、PX4 SITL 和 Gazebo 的组合通常建议在 Ubuntu 20.04、虚拟机或 Ubuntu 主机中运行。

> PX4 不同版本使用的仿真器命令不同。较旧版本常见 `make px4_sitl gazebo`，新版本可能使用 `make px4_sitl gz_x500`。请以所使用 PX4 版本的官方文档为准。

## 安装依赖

### 1. 安装 ROS Noetic

按照 ROS 官方文档安装 ROS Noetic Desktop Full。安装后执行：

```bash
source /opt/ros/noetic/setup.bash
```

### 2. 安装 MAVROS

```bash
sudo apt update
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras
```

安装 GeographicLib 数据集：

```bash
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
```

### 3. 下载 PX4-Autopilot

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
```

依赖安装完成后重新打开终端。

## 编译本项目

进入工作空间根目录：

```bash
cd ~/px4_offboard_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

如果每次打开终端都要使用该工作空间，可以将下面一行加入 `~/.bashrc`：

```bash
source ~/px4_offboard_ws/devel/setup.bash
```

## 运行仿真

至少需要两个终端。初次使用建议先运行位置控制节点。

### 终端 1：启动 PX4 SITL 和 Gazebo

旧版 PX4 / Gazebo Classic 常见命令：

```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo
```

新版 PX4 / Gazebo Harmonic 常见命令：

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

### 终端 2：运行位置控制

```bash
source /opt/ros/noetic/setup.bash
source ~/px4_offboard_ws/devel/setup.bash
roslaunch offboard_control offboard_position.launch
```

节点会持续发送 setpoint，尝试切换到 `OFFBOARD` 模式并解锁。默认轨迹为：

| 时间 | 目标位置 `(x, y, z)` |
| --- | --- |
| 0-10 秒 | `(0, 0, 2)` |
| 10-20 秒 | `(5, 0, 3)` |
| 20-30 秒 | `(5, 5, 3)` |
| 30-40 秒 | `(0, 5, 3)` |
| 40 秒以后 | `(0, 0, 2)` |

坐标使用 MAVROS 本地 ENU 坐标系：X 向东、Y 向北、Z 向上。

### 终端 2：运行 PD 控制

停止位置控制节点后，再运行：

```bash
source /opt/ros/noetic/setup.bash
source ~/px4_offboard_ws/devel/setup.bash
roslaunch offboard_control offboard_pd.launch
```

不要同时运行两个 Offboard 控制节点，否则它们会向 PX4 发送互相冲突的 setpoint。

## PD 参数

参数文件：`src/offboard_control/config/pd_params.yaml`

| 参数 | 默认值 | 含义 |
| --- | ---: | --- |
| `Kp_pos` | `0.95` | 位置误差到期望速度的比例增益 |
| `Kp_vel` | `1.8` | 速度误差到期望加速度的比例增益 |
| `Kd_vel` | `0.4` | 速度误差微分增益，用于增加阻尼 |
| `max_vel` | `3.0` | 最大期望速度，单位 m/s |
| `max_accel` | `5.0` | 最大期望加速度，单位 m/s² |
| `max_tilt` | `0.5` | 最大倾斜角，单位 rad，约 28.6° |
| `hover_thrust` | `0.5` | 水平悬停时的归一化推力 |

建议先保持较低的 `max_vel` 和 `max_tilt`，确认方向和坐标系正确后再逐步调参。

## 常用检查命令

检查 MAVROS 是否连接到 PX4：

```bash
rostopic echo /mavros/state
```

正常时应看到：

```text
connected: True
```

查看当前位置：

```bash
rostopic echo /mavros/local_position/pose
```

查看当前速度：

```bash
rostopic echo /mavros/local_position/velocity_local
```

查看节点和话题连接关系：

```bash
rqt_graph
```

## 常见问题

### MAVROS 一直显示未连接

确认 PX4 SITL 已启动，并检查 launch 文件中的 FCU 地址：

```text
udp://:14540@127.0.0.1:14557
```

不同 PX4 版本或启动方式可能使用不同 UDP 端口。如果没有连接，应查看 PX4 启动日志中的 MAVLink 端口配置并修改 launch 文件。

### 无法切换到 OFFBOARD

PX4 要求切换模式前已经持续收到 setpoint，并要求 Offboard 期间 setpoint 发布频率高于 2 Hz。本项目在切换模式前会先发送一段时间的 setpoint。

另外检查：

- `/mavros/state` 中 `connected` 是否为 `True`。
- PX4 是否存在 preflight check 报错。
- 本地位置估计是否有效。
- 是否有另一个节点同时发布 setpoint。

### PD 节点解锁后无法起飞

优先检查 `hover_thrust`。默认值 `0.5` 只是仿真初值，不同机型需要标定。如果数值太小，无人机无法离地；过大则可能快速上冲。

### 编译时找不到 ROS 或 MAVROS

确认已经加载 ROS 环境并安装依赖：

```bash
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```

## 安全说明

- 先在 Gazebo 中验证，不要直接在真实无人机上运行。
- PD 节点会直接发送姿态和推力目标，错误参数可能造成快速倾斜或冲高。
- 测试时不要同时运行两个控制节点。
- 真机测试必须加入遥控器接管、失控保护、地理围栏和紧急上锁方案。
- 真机使用前必须重新确认 ENU、NED、FLU 和 FRD 坐标转换。

## 参考资料

- [PX4 Offboard Mode](https://docs.px4.io/main/en/flight_modes/offboard.html)
- [PX4 ROS/MAVROS Offboard 示例](https://docs.px4.io/main/en/ros/mavros_offboard_cpp.html)
- [MAVROS](https://github.com/mavlink/mavros)
- [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot)

## License

BSD-3-Clause
