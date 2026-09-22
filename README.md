# RoboMaster 算法组第二周任务

本仓库包含 ROS 2 C++ 信号滤波与直流电机机械模型仿真，并使用 Docker Compose 为两个任务提供相互独立的运行环境。

## 项目结构

```text
.
├── Dockerfile
├── compose.yaml
├── test1_ws/
│   └── src/task1_filters/
└── test2_ws/
    └── src/task2_motor/
```

## 环境

- ROS 2 Jazzy
- C++ / `ament_cmake`
- Docker Compose

构建并启动容器：

```bash
docker compose up -d --build
```

两个任务分别使用独立容器与 ROS Domain ID：

| 任务 | Compose 服务 | 容器名 | ROS Domain ID |
|---|---|---|---:|
| 任务1 | `task1` | `robomaster-week2-task1` | 21 |
| 任务2 | `task2` | `robomaster-week2-task2` | 22 |

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

## 许可证

本项目使用 MIT License。
