# rosbag reply tool

ROS 2 bag 图形化回放与 ESKF 定位调试工具。播放器支持拖动进度、精确时间跳转、暂停/继续、
独立 `ROS_DOMAIN_ID` 多开，以及 ESKF 同进程检查点恢复。ESKF 调试布局同时提供 RViz、
实时过程量浮窗和日志容量管理。

## 仓库内容

- `rosbag_progress_player.py`：主播放器和进度条界面。
- `progress_player/eskf_compare.yaml`：ESKF 回放 profile。
- `progress_player/launch_eskf_multi_replay.sh`：三包多 Domain 桌面布局。
- `progress_player/eskf_live_inspector.py`：传感器状态、ESKF 过程量和日志管理浮窗。
- `progress_player/managed_stack_supervisor.py`：播放器消失时清理整棵 ESKF/RViz 进程树。
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

启动前脚本会检查计划使用的全部 Domain。任一 Domain 存在旧节点时，会在创建窗口前列出占用
节点并整体退出，避免出现“进度条有三个、RViz 只有一个”的半启动桌面。可停止旧节点，或者：

```bash
ESKF_MULTI_BASE_DOMAIN=190 \
  /home/hulk/ros2bag/progress_player/launch_eskf_multi_replay.sh --select
```

RViz 首次映射后可能恢复自身保存的默认窗口尺寸。多开脚本会在全部 RViz 启动完成后统一执行两次
最终布局，确保第一列不会停留在默认 `1400×900` 并遮挡其他列。

每列从上到下对应同一个 bag/Domain 的 RViz、浮动过程量窗口和进度条。在启动终端按
`Ctrl+C` 会结束该次多包回放创建的播放器、ESKF、adapter、RViz 和浮窗。
`ros2 bag play` 和 ESKF/RViz 托管栈分别经过 supervisor。即使后台终端或播放器被外部直接
关闭，两个 supervisor 也会检测父 PID 消失并清理各自的独立进程组。

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
python3 progress_player/test_managed_stack_supervisor.py
python3 -m py_compile rosbag_progress_player.py progress_player/eskf_live_inspector.py
bash -n progress_player/launch_eskf_multi_replay.sh
```

当前 ESKF 检查点功能位于 `feature/eskf-replay-checkpoint` 分支。

多包启动器会先运行 `progress_player/ensure_eskf_replay_build.sh`。守卫对 ESKF
源码、CMake、Production 参数、运行参数生成脚本、标定文件、RViz 插件源码以及最终
安装产物计算 SHA-256。输入和产物均未变化时直接复用已有编译；任一内容变化、产物
缺失或状态文件损坏时才重新执行 `prepare_eskf_compare.sh`。本地状态保存在
`progress_player/build_state/`，不提交到 Git。

这里的“复用编译”不等于跨进程恢复 checkpoint：`FullReplayCheckpoint` 当前保存在
ESKF 进程内存中。关闭整套回放后重新启动，仍需先完整播放才能建立本进程的精确状态；
工具不会仅凭二进制未变化就把旧内存状态标记为可恢复。

该专项 profile 默认使用 `progress_player/ordinal_replay_reader.py`，不再调用
`ros2 bag play` 给 ESKF 注入数据。reader 首次顺序读取 storage，给每条原始 bag 记录
分配全局 `bag_ordinal`；IMU、GPS、Odom、Cmd 通过单一
`/eskf/replay_ingress` 发送，并等待相同 `(epoch, bag_ordinal)` 的 ACK 后才发送下一条。
这保证相同时间戳记录仍按 storage 原始顺序执行。

`eskf_compare.yaml` 中必须同时设置：

```yaml
checkpoint_restore: true
ordinal_reader: true
```

普通 profile 不设置 `ordinal_reader` 时仍使用标准 `ros2 bag play`。

专项模式第一次必须完整播放到 EOF。reader 收到最后一条 ESKF ACK 后才请求封存过程
日志和冻结 checkpoint；封存 ACK 返回前，进度条拒绝恢复。之后每次拖动或指定时刻会
自动执行三个新 replay epoch，在同一 `target_ordinal` 停住并比较：

- `FullReplayCheckpoint` 的 104 个可恢复字段，包括四个嵌套状态对象、各输入队列、
  GNSS/ground/OOSM 历史、诊断/trust 状态和最后一次 computation trace；
- 15×15 协方差的 225 个 binary64 元素；
- 本 epoch 实际发布的输出轨迹（ordinal、时间、位置和姿态）。

checkpoint 每个字段由 C++ 使用带类型和长度的固定 big-endian 编码生成独立 SHA-256，
再由工具对字段名/摘要映射生成完整状态 SHA-256；`-0` 统一为 `+0`，checkpoint 内若有
NaN/Inf 则保留其 IEEE-754 位模式参与比较。协方差和轨迹仍拒绝非有限值。
构建测试会核对 checkpoint 声明与 canonical manifest，新增字段未编码会直接失败。
结果保存在 `determinism_reports/target_<ns>_<time>.json`。只有三类哈希
全部一致，且首次日志字节数和 checkpoint 数量在三轮前后不变，界面才显示
`bit-exact 通过`；失败报告包含首个不同字段或矩阵/轨迹下标。
