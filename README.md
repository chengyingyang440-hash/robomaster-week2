# RoboMaster 算法组第二周任务

本仓库包含 ROS 2 C++ 信号滤波、直流电机机械模型仿真、PID 转速闭环控制和双环 PID 角度控制，并使用 Docker Compose 提供相互独立的运行环境。

## 项目结构

```text
.
├── Dockerfile
├── compose.yaml
├── test1_ws/
│   └── src/task1_filters/
├── test2_ws/
│   └── src/task2_motor/
├── test3_ws/
│   └── src/task3_pid/
└── test3_advanced_ws/
    └── src/task3_dual_pid/
```

## 环境

- ROS 2 Jazzy
- C++ / `ament_cmake`
- Docker Compose

构建并启动容器：

```bash
docker compose up -d --build
```

三个任务分别使用独立容器与 ROS Domain ID：

| 任务 | Compose 服务 | 容器名 | ROS Domain ID |
|---|---|---|---:|
| 任务1 | `task1` | `robomaster-week2-task1` | 21 |
| 任务2 | `task2` | `robomaster-week2-task2` | 22 |
| 任务3 | `task3` | `robomaster-week2-task3` | 23 |
| 任务3进阶 | `task3_advanced` | `robomaster-week2-task3-advanced` | 24 |

## 任务1：信号滤波

`signal_generator` 以 1000 Hz 发布带高斯噪声的 20 Hz 正弦信号，`signal_filter` 同时计算一阶低通滤波和窗口长度为 5 的中值滤波。

主要话题：

| 话题 | 含义 |
|---|---|
| `/signal/noisy` | 原始含噪信号 |
| `/signal/low_pass` | 一阶低通滤波结果 |
| `/signal/median` | 中值滤波结果 |

构建：

```bash
docker compose exec task1 bash
cd /workspace/test1_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

分别运行：

```bash
ros2 run task1_filters signal_generator
ros2 run task1_filters signal_filter
```

## 任务2：电机模拟器

电机模拟器使用隐藏电气部分后的机械模型：

```text
J * dω/dt = T_e - T_L - B * ω
dθ/dt = ω
```

当前参数：

| 参数 | 数值 |
|---|---:|
| 转动惯量 `J` | `0.01 kg·m²` |
| 粘性阻尼系数 `B` | `0.1 N·m·s/rad` |
| 负载力矩 `T_L` | `0 N·m` |
| 状态更新步长 | `0.001 s`（1000 Hz） |
| 控制力矩发布周期 | `0.002 s`（500 Hz） |

节点和话题：

| 节点 | 作用 |
|---|---|
| `torque_source` | 发布测试用控制力矩 |
| `motor_simulator` | 计算并发布转速和角度 |

| 话题 | 含义 |
|---|---|
| `/motor/torque_cmd` | 控制力矩，单位 N·m |
| `/motor/angular_velocity` | 角速度，单位 rad/s |
| `/motor/angle` | 累计角度，单位 rad |

构建并一键运行：

```bash
docker compose exec task2 bash
cd /workspace/test2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch task2_motor motor_demo.launch.py
```

测试信号为：

```text
0～2 s：0 N·m
2～5 s：1 N·m
5 s 后：0 N·m
```

在 `1 N·m` 恒定力矩下，理论稳态角速度为：

```text
ω_ss = (T_e - T_L) / B = 10 rad/s
```

实测角速度稳定在约 `10 rad/s`；撤去力矩后在阻尼作用下衰减至零，最终累计角度约为 `30 rad`。节点以 1000 Hz 更新状态，并将日志限频到 2 Hz，便于终端演示。

## 任务3：PID 转速闭环控制

任务3复用任务2的电机机械模型，并由 `speed_controller` 根据目标转速和实际转速计算控制力矩：

```text
目标转速 ──> PID 控制器 ──> 控制力矩 ──> 电机模拟器
                 ↑                         │
                 └────── 实际转速 <────────┘
```

PID 控制规律为：

```text
error = target_velocity - current_velocity
torque_cmd = Kp * error
           + Ki * integral(error * dt)
           + Kd * (error - previous_error) / dt
```

当前实验参数：

| 参数 | 数值 |
|---|---:|
| 目标转速 | `10 rad/s` |
| `Kp` | `0.2` |
| `Ki` | `0.1` |
| `Kd` | `0.0` |
| 电机状态更新周期 | `0.001 s`（1000 Hz） |
| PID 控制周期 | `0.002 s`（500 Hz） |

节点和话题：

| 节点 | 作用 |
|---|---|
| `speed_controller` | 订阅实际转速，通过 PID 计算并发布控制力矩 |
| `motor_simulator` | 接收控制力矩，更新并发布电机转速与角度 |

| 话题 | 含义 |
|---|---|
| `/motor/torque_cmd` | PID 输出的控制力矩，单位 N·m |
| `/motor/angular_velocity` | 电机实际角速度，单位 rad/s |
| `/motor/angle` | 电机累计角度，单位 rad |

构建并一键运行：

```bash
docker compose exec task3 bash
cd /workspace/test3_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch task3_pid pid.launch.py
```

纯比例控制使用 `Kp=0.2`、`Ki=0`、`Kd=0` 时，实际转速稳定在约 `6.667 rad/s`。这是因为稳态时需要保留非零误差，才能产生抵消粘性阻尼的力矩：

```text
0.2 * (10 - ω) = 0.1 * ω
ω = 6.667 rad/s
```

加入积分项后，积分输出逐渐补偿阻尼力矩，实际转速最终稳定在目标值 `10 rad/s`，消除了稳态误差。控制节点每 500 ms 输出一次目标转速、实际转速、误差和控制力矩，便于观察调参结果。

使用 Foxglove 可视化时，在另一个终端进入 `task3` 容器并启动 bridge：

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/test3_ws/install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

宿主机通过 `ws://localhost:8767` 连接，并绘制 `/motor/torque_cmd`、`/motor/angular_velocity` 和 `/motor/angle`，即可观察电机输入与输出数据。

## 任务3进阶：双环 PID 角度控制

进阶任务将角度环和速度环拆成两个 ROS 2 节点：

```text
目标角度 ──> angle_controller ──> 目标速度 ──> speed_controller ──> 控制力矩
                  ↑                                  ↑
                  │                                  │
               实际角度                           实际速度
                  └────────── motor_simulator ───────┘
```

`angle_controller` 是外环，订阅 `/motor/angle` 并发布 `/motor/velocity_cmd`；`speed_controller` 是内环，订阅目标速度和实际速度并发布 `/motor/torque_cmd`。

当前参数：

| 控制环 | `Kp` | `Ki` | `Kd` |
|---|---:|---:|---:|
| 角度外环 | 1.0 | 0.0 | 0.0 |
| 速度内环 | 0.2 | 0.5 | 0.0 |

目标角度为 `1.5π rad`，目标速度限制为 `±4 rad/s`。从 `0 rad` 出发时，控制器选择 `+4.712 rad` 的优弧，而不是 `-1.571 rad` 的劣弧，最终稳定在约 `4.712 rad`。

构建并运行：

```bash
docker compose exec task3_advanced bash
cd /workspace/test3_advanced_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch task3_dual_pid dual_pid.launch.py
```

使用 Foxglove 可视化时，在另一个终端进入 `task3_advanced` 容器并运行：

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

宿主机使用 `ws://localhost:8768` 连接，可绘制以下数据：

```text
/motor/angle.data
/motor/angular_velocity.data
/motor/velocity_cmd.data
/motor/torque_cmd.data
```

## 许可证

本项目使用 MIT License。
