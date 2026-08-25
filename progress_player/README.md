# Stateful ROS 2 bag replay

完整安装、启动、过程量和日志管理说明见仓库根目录 [`README.md`](../README.md)。

`rosbag_progress_player.py` has two seek modes:

- without a profile, seek is the native rosbag2 seek and does not reset nodes;
- with a profile, every seek restarts the managed command and replays all recorded
  inputs from the bag start to reconstruct state.

Run the ESKF profile through the project wrapper:

```bash
bash /home/hulk/mow_mow_agent/mowmow/docs/eskf_fusion/debug/run_eskf_compare.sh --progress-player
```

The reconstruction rate defaults to 10x. The player pauses on the first `/clock`
sample at or after the target and reports the measured overshoot. It deliberately
does not seek backward after pausing, because that would make the node state newer
than ROS time.

Only values present in the bag can be reconstructed. Parameters, service calls,
files, device state, or process memory absent from the recording cannot be restored.

The floating inspector implementation lives in this replay-tool repository:

```bash
python3 /home/hulk/ros2bag/progress_player/eskf_live_inspector.py --help
```
