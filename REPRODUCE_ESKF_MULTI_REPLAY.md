# ESKF 多包回放复现说明

本文锁定 2026-08-25 已通过真实桌面验收的三包回放环境。rosbag 数据不进入 Git；
代码、配置、校验值和启动步骤进入远端分支。

## 1. 代码版本

按以下固定路径检出，现有脚本不依赖调用终端的当前目录：

```bash
mkdir -p /home/hulk/mow_mow_agent /home/hulk
git clone --branch feature/eskf-fusion \
  git@gitee.com:yosemite---shanghai---robot/mowmow.git \
  /home/hulk/mow_mow_agent/mowmow
git -C /home/hulk/mow_mow_agent/mowmow submodule update --init --recursive

git clone --branch feature/eskf-replay-checkpoint \
  https://github.com/Smashwinny/ros2bag_replytool.git /home/hulk/ros2bag
```

验收时锁定的依赖提交：

- mowmow：`d82a7fbc`（`feature/eskf-fusion`）
- mowmow_location：`5edbf39c74384eb787d009a6ae9a237ee103a1d4`
- ros2bagtool：以本文件所在的 `feature/eskf-replay-checkpoint` 分支 HEAD 为准

检出后必须确认：

```bash
git -C /home/hulk/mow_mow_agent/mowmow rev-parse HEAD
git -C /home/hulk/mow_mow_agent/mowmow/mowmow_location rev-parse HEAD
git -C /home/hulk/ros2bag rev-parse --abbrev-ref HEAD
```

## 2. 系统依赖与 ESKF 准备

```bash
sudo apt update
sudo apt install ros-humble-rosbag2 ros-humble-rosbag2-interfaces \
  ros-humble-rviz2 python3-yaml python3-tk wmctrl xdotool

source /opt/ros/humble/setup.bash
cd /home/hulk/mow_mow_agent/mowmow
# 顶层 install 中需先存在 avengers_msgs 和 fusion_location_interface。
bash docs/eskf_fusion/debug/prepare_eskf_compare.sh
```

`prepare_eskf_compare.sh` 会生成 production-equivalent 回放参数，并编译安装 ESKF 与
debug RViz 插件。缺少两个自定义消息 install 时脚本会 fail closed，不会用不完整环境继续。

多包启动器会自动用内容指纹复用未变化的编译产物。首次完整播放结束后需等待进度窗口
显示正在建立/已建立持久化缓存；关闭并再次启动相同 bag 后，状态栏应显示“只读缓存
回放”，此时拖动进度条不会启动 ESKF 或增加 `eskf_process.jsonl`。

## 3. 默认数据清单

将三份 bag 放到下列精确路径。使用 `sha256sum` 核对每个文件：

```text
1ddeb1e9f527903f0a3302911ee05524572b957e535555df83a7ba58a0ec2f3c  /home/hulk/ros2bag/rosbag2_2026_08_07-15_56_44/rosbag2_2026_08_07-15_56_44_0.db3
be449634b63fb6757be3cda60ba007632cefaf6191d597a1c4a04cbeb2cf10b2  /home/hulk/ros2bag/rosbag2_2026_08_07-15_56_44/metadata.yaml
7499cfe93cc72682265c6c3c6ce12e4d8391f36989fa1ac5fde3d43654d1308d  /home/hulk/ros2bag/rosbag2_2026_08_07-15_56_44/target_pos_all_3.yaml

550f0148affe956c16b08ef9fbe192cb42461cfd5f4fee3416784dde1d34f21d  /home/hulk/ros2bag/rosbag2_2026_08_07-16_40_39/rosbag2_2026_08_07-16_40_39_0.db3
98a6711e0929222757e42143bf919d1e4d7222396e6a733f46e47bfaf73b934e  /home/hulk/ros2bag/rosbag2_2026_08_07-16_40_39/metadata.yaml
bc7e1986b56d10b11100daf774b86b001c7815eeceb3751b1a89176d57cc2fe5  /home/hulk/ros2bag/rosbag2_2026_08_07-16_40_39/target_pos_all_4.yaml

c97af386a33bd0fbf6969a25e1097957e0a28365bed3fa87dea083dc8fc70042  /home/hulk/ros2bag/bag/rosbag2_2026_08_13-15_08_48/rosbag2_2026_08_13-15_08_48_0.db3
4f01be6bc102f474ca95f333987cf96423b6158a8c641d0b72409fbe10b48627  /home/hulk/ros2bag/bag/rosbag2_2026_08_13-15_08_48/metadata.yaml
da91248eeb0c8615500d668546e7dcca7d762826ddecab632850790d14d39b02  /home/hulk/ros2bag/bag/rosbag2_2026_08_13-15_08_48/target_pos_all_3.yaml
```

地图映射来自旧验证脚本的显式关系；`progress_player/eskf_compare.yaml` 对未知 bag
或缺失 YAML 会拒绝启动。

## 4. 启动与验收

从图形桌面终端执行：

```bash
source /opt/ros/humble/setup.bash
/home/hulk/ros2bag/progress_player/launch_eskf_multi_replay.sh
```

预期结果：

- 默认创建三个连续 ROS Domain，且 `ROS_LOCALHOST_ONLY=1`；
- 三列分别显示同名 RViz、浮动 Sensor Trust 状态条和进度播放器；
- 初始暂停，可独立播放、拖动和改变倍速；
- 黄色参考地图分别来自 target3、target4、target3；
- 播放后 `/legacy/fusion_location`、`/eskf/fusion_location_xy` 和三类传感器状态更新；
- 向过去拖动时使用同进程 ESKF 检查点恢复，过程量保存在
  `progress_player/runs/<run-id>/eskf_process.jsonl`。

静态回归测试：

```bash
cd /home/hulk/ros2bag
python3 -m unittest progress_player/test_profile.py
bash -n progress_player/launch_eskf_multi_replay.sh
python3 -m py_compile rosbag_progress_player.py
```

验收边界：Git 不包含 bag 本体、构建目录和运行日志；复现者必须取得上面校验值一致的
原始 bag。检查点是进程内状态，退出回放后保留的是完整 JSONL 过程证据，不是可跨进程
加载的磁盘检查点。
