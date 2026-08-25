# ROS 2 Bag 进度回放工具

这是一个面向 ROS 2 Humble 的桌面回放工具。它为 `ros2 bag play` 增加可拖动的
时间进度条，并提供播放、暂停、跳转、倍速、Domain ID 和仅本机通信配置。

工具通过 rosbag2 自带的 `seek`、`pause`、`resume` 和 `set_rate` 服务控制播放，
不会修改原始 bag 文件。

## 功能

- 显示当前播放时间、总时长和实时进度。
- 播放中或暂停时向前、向后拖动进度条。
- 一键前进或后退 10 秒。
- 支持 0.25、0.5、1、2、5、10 倍速。
- 播放到末尾后自动回到开头，控制服务不会退出。
- 自动以 30 Hz 发布 `/clock`。
- 在界面配置 `ROS_DOMAIN_ID`（0～232）。
- 在界面启用或关闭 `ROS_LOCALHOST_ONLY=1`。
- 网络设置同时作用于 rosbag2 播放器和 GUI 控制节点。
- 可通过 `--profile` 托管有状态算法；拖动时重启算法并从 bag 起点重建状态。

## 环境要求

- Ubuntu 22.04
- ROS 2 Humble，已安装 `rosbag2` 和 `rosbag2_interfaces`
- Python 3
- Python 模块：`rclpy`、`PyYAML`、`tkinter`

Ubuntu/ROS 2 Humble 常用安装命令：

```bash
sudo apt update
sudo apt install ros-humble-rosbag2 ros-humble-rosbag2-interfaces \
  python3-yaml python3-tk
```

运行前加载 ROS 环境：

```bash
source /opt/ros/humble/setup.bash
```

如果 bag 包含自定义消息，还需要加载对应工作空间，例如：

```bash
source /path/to/your_ws/install/setup.bash
```

## 启动

在图形界面中选择 bag：

```bash
python3 rosbag_progress_player.py
```

启动时直接指定 bag 目录或 `metadata.yaml`：

```bash
python3 rosbag_progress_player.py /path/to/your_bag
```

ESKF 对比统一入口：

```bash
bash /home/hulk/mow_mow_agent/mowmow/docs/eskf_fusion/debug/run_eskf_compare.sh --progress-player
```

同时展示三个相互隔离的 ESKF 数据包，并自动排列 RViz2 与进度条：

```bash
bash /home/hulk/ros2bag/progress_player/launch_eskf_multi_replay.sh
```

多开前应先使用已有单包包装入口验证；例如：

```bash
ESKF_COMPARE_DOMAIN_ID=186 bash \
  /home/hulk/mow_mow_agent/mowmow/docs/eskf_fusion/debug/run_eskf_compare_20260813_150848.sh \
  --progress-player
```

也可以指定一到三个包；顺序就是桌面从左到右的顺序：

```bash
bash /home/hulk/ros2bag/progress_player/launch_eskf_multi_replay.sh \
  /path/to/bag_a /path/to/bag_b /path/to/bag_c
```

默认使用 P92 的 Domain 182～184；可通过 `ESKF_MULTI_BASE_DOMAIN` 修改。每列使用独立 Domain，
上方 RViz2 和下方进度条标题含相同的序号、bag 名称和 Domain；播放器初始为暂停状态。

ESKF profile 会复用旧对比脚本的 topic 白名单和 remap：`/odometry` 映射到
`/bag/odometry`，包内 `/fusion_location` 映射到 `/legacy/fusion_location`。受管脚本从 bag
目录或相邻 `map/` 目录解析唯一的 `target_pos_all_3.yaml`/`target_pos_all_4.yaml`；找不到或
存在多个候选时拒绝启动，不能静默套用其他包的地图。

选择的目录必须包含 rosbag2 生成的 `metadata.yaml`。

## 使用方法

1. 设置 `ROS_DOMAIN_ID`，范围为 0～232。
2. 只允许本机 ROS 节点通信时，勾选“仅本机通信”。
3. 点击“选择 Bag”，选择包含 `metadata.yaml` 的目录。
4. 等待状态栏显示“已暂停，可拖动进度条”。
5. 点击“播放”，或者先拖动进度条到需要检查的时间点。
6. 播放期间修改网络设置后，点击“应用并重启播放”。新配置会生效，bag 会从开头重新启动。

界面状态栏会显示当前真正生效的 Domain 和通信模式，例如：

```text
正在播放（Domain 77 · 本机）
```

## 与其他 ROS 节点配合

播放器会发布 `/clock`。需要使用 bag 时间的 RViz、定位、建图或滤波节点必须启用
模拟时间：

```bash
ros2 run your_package your_node --ros-args -p use_sim_time:=true
rviz2 --ros-args -p use_sim_time:=true
```

这些节点的 `ROS_DOMAIN_ID` 和 `ROS_LOCALHOST_ONLY` 必须与界面状态栏显示的配置一致。

## 注意事项

- 向过去拖动时间后，ROS 时间会发生倒退。
- ESKF、建图、定位等有内部历史状态的节点不会因原生 `seek` 自动复位。
- `checkpoint_restore: true` 的专项 profile 会先暂停，向
  `/eskf/replay_restore_request` 发送目标绝对时间，等待 ESKF 在同一进程恢复不晚于目标的
  完整检查点，再把 bag reader 移到检查点之后并以 1x 补放到目标。该路径不重启节点。
- ESKF 专项回放将逐事件计算过程量写入
  `progress_player/runs/<run-id>/eskf_process.jsonl`；回放退出后文件保留，包含状态、完整协方差、
  predict 的 F/G/Qc/Phi/Qd，以及 update 的 residual/H/R/S/K/delta/Joseph/reset 矩阵。
- 检查点是进程内对象，关闭 ESKF 进程后不能再次恢复；JSONL 是回放后调试证据，不是磁盘检查点。
- rosbag2 的时间 seek 不能区分具有相同时间戳的多条 DDS 记录，因此当前专项恢复保证 ESKF
  内部检查点原子还原，但尚不宣称任意同时间戳 bag 游标都能逐消息 bit-exact 复现。
- 单纯查看 RViz 或无状态话题通常可以直接拖动。
- 使用 profile 时，工具会重启受管节点并从 bag 起点高速回放到目标；这只能重建 bag
  已记录输入所决定的状态，不能恢复未记录参数、服务调用、文件或设备状态。
- 重建在第一个不早于目标的 `/clock` 样本暂停，并显示越界毫秒数；不会再向后 seek，
  以免算法状态比 ROS 时间更新。
- 若 bag 缺少自定义消息包，rosbag2 会忽略无法解析的相关话题。

## 常见问题

### 点击播放后进度条不动

本工具使用与 rosbag2 `/clock` 发布端兼容的 `BEST_EFFORT` QoS。若仍不更新，请确认：

```bash
ros2 topic info /clock --verbose
ros2 topic echo /clock --once
```

同时检查是否存在其他播放器占用了同名 `/rosbag2_player` 服务。

### 一直显示“正在等待播放器服务”

确认其他终端和界面使用相同的 Domain/local 配置，并检查：

```bash
ros2 service list | grep rosbag2_player
```

### 关闭窗口

正常关闭窗口时，工具会向它启动的 rosbag2 进程发送停止信号，不会修改或删除 bag。
