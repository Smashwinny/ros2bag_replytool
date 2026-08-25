# rosbag reply tool

ROS 2 bag 图形化回放与 ESKF 定位调试工具。播放器支持拖动进度、精确时间跳转、暂停/继续、
独立 `ROS_DOMAIN_ID` 多开，以及 ESKF 同进程检查点恢复。ESKF 调试布局同时提供 RViz、
实时过程量浮窗和日志容量管理。

## 仓库内容

- `rosbag_progress_player.py`：主播放器和进度条界面。
- `progress_player/eskf_compare.yaml`：ESKF 回放 profile。
- `progress_player/launch_eskf_multi_replay.sh`：三包多 Domain 桌面布局。
- `progress_player/eskf_live_inspector.py`：传感器状态、ESKF 过程量和日志管理浮窗。
- `ROSBAG_PROGRESS_PLAYER.md`：通用播放器参数说明。
- `REPRODUCE_ESKF_MULTI_REPLAY.md`：完整环境、数据和复现步骤。

## 环境准备

```bash
cd /home/hulk/mow_mow_agent/mowmow
bash docs/eskf_fusion/debug/prepare_eskf_compare.sh
source /opt/ros/humble/setup.bash
```

多窗口排版依赖 `rviz2`、Tk、`wmctrl`、`xdotool`、`xrandr` 和图形桌面环境。

## 三包并排回放

```bash
/home/hulk/ros2bag/progress_player/launch_eskf_multi_replay.sh
```

无参数启动默认三个包。需要图形化选择 1～3 个已登记数据包时：

```bash
/home/hulk/ros2bag/progress_player/launch_eskf_multi_replay.sh --select
```

也可以直接传入 1～3 个 bag 路径。脚本会先并行启动全部选中回放栈，再统一寻找和排列窗口，
不会因为等待第一个 RViz 而推迟第二、第三个 Domain。

每列从上到下对应同一个 bag/Domain 的 RViz、浮动过程量窗口和进度条。在启动终端按
`Ctrl+C` 会结束该次多包回放创建的播放器、ESKF、adapter、RViz 和浮窗。

## 单包回放

```bash
bash /home/hulk/mow_mow_agent/mowmow/docs/eskf_fusion/debug/run_eskf_compare.sh \
  --progress-player
```

可通过 `ESKF_COMPARE_BAG`、`ESKF_COMPARE_TARGET_YAML` 和
`ESKF_COMPARE_DOMAIN_ID` 指定 bag、参考 YAML 和 Domain。推荐优先使用项目已有的包专用包装
脚本，避免 bag/YAML 配错。

## 精确跳转与状态恢复

- 拖动进度条可跳转。
- 点击时钟图标可输入秒、`MM:SS.mmm` 或 `HH:MM:SS.mmm`，例如 `00:00:05.250`。
- profile 模式优先恢复目标时间之前的 ESKF 同进程检查点，再补放到目标时间。
- 跳转会同步裁剪或清空 RViz 历史轨迹，避免显示目标时间之后的旧路径。

## 过程量和日志按钮

浮窗默认显示 IMU/GNSS/Odom 可信状态、位置、速度、协方差、NIS、队列和 OOSM 计数；点击
`▾` 展开 F/G/Q、H/R/S/K、残差、bias、`delta_x`、Joseph 和 reset 摘要。

底部显示当前 Domain 日志大小与全部日志占用：

- `📂 日志`：打开 `/home/hulk/ros2bag/progress_player/runs/`。
- `清理`：二次确认后永久删除历史运行目录。
- 当前 Domain 和其他带 `.active` 标记的运行目录不会被删除。

完整矩阵日志可能增长到数 GB。定位问题时保留完整模式，结束后可通过浮窗清理；删除不可恢复。

## 测试

```bash
cd /home/hulk/ros2bag
python3 progress_player/test_profile.py
python3 progress_player/test_eskf_live_inspector.py
python3 -m py_compile rosbag_progress_player.py progress_player/eskf_live_inspector.py
bash -n progress_player/launch_eskf_multi_replay.sh
```

当前 ESKF 检查点功能位于 `feature/eskf-replay-checkpoint` 分支。
