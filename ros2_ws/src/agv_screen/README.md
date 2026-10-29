# 正式比赛屏幕通信

`screen_node` 用 Python/rclpy 实现，独占屏幕 USB 串口。现有 `工创赛屏幕驱动/`、HMI 和测试脚本不修改；运行时不依赖源码目录内的示例程序。

已实现：人工禁行路径、start/stop 选择、OK 后至少两秒且执行端就绪时启动、显示本轮已确认的扫码结果。第四阶段成功抓取/放置统计尚未接入，比赛结束保留任务码，不发送虚构的次数。

## 构建与启动

```bash
source /opt/ros/humble/setup.bash
cd /root/工创/ros2_ws
colcon build --symlink-install --packages-up-to agv_screen
source /root/工创/ros2_ws/install/setup.bash
ros2 launch agv_screen screen_mission.launch.py
```

也可直接使用仓库脚本（构建一次后）：

```bash
bash /root/工创/tools/start_screen_mission.sh
```

此入口启动 **mission 和屏幕通信两个节点**，不是尚未完成的整套比赛启动器。不要同时再启动另一份 mission 或屏幕测试脚本。C 板、规划器、雷达及视觉执行端沿用各自入口；全部真实执行端就绪后比赛才可开始。当前缺失视觉/雷达执行端时，提交屏幕配置后会等待初始化，超时进入 FAULT，不假装比赛已开始。

屏幕通信配置位于 `config/screen.yaml`，默认已验证的 CP2102 稳定路径、9600、8N1。替换配置可运行：

```bash
ros2 launch agv_screen screen_mission.launch.py screen_config:=/绝对路径/screen.yaml
```

## 操作流程

1. 启动进入 `WAIT_SETUP`，屏幕切到路径页并清除旧任务码。核对按钮是否保留上轮高亮，必要时手动取消。
2. 选择不能走的路径，按 OK。屏幕节点打印本轮人工禁行列表；空选择也有效。路径 1 对应 7↔4，封锁两个方向。
3. 选择 start/stop，按 OK。屏幕节点把完整配置和启动意图提交 `/mission/configure_and_start`；start1/stop1 对应规划节点 1，start2/stop2 对应节点 3。
4. mission 按选择的起点和朝向初始化执行端。从接受配置时开始计时，至少两秒且系统 READY 后自动启动一次，打印“比赛开始”。若初始化超过两秒，则在真正就绪后开始；若初始化失败，则不启动。不用另调 `/mission/start`。
5. 开局雷达扫描成功后，mission 统一将人工禁行与雷达禁行取并集、去重，发布一份地图；雷达不能清除人工禁行。规划器确认地图后进入现有比赛阶段。
6. 扫码程序输出 `/qr/result`；mission 校验并在扫码阶段确认后发布 `/mission/task_info`。屏幕显示两行任务码，如 `156+123`、`516+231`。无效任务码不显示，不把旧轮码当作新轮结果。

原扫码节点的“连续五次一致、只输出一次并释放相机”行为不变。应先启动调度，再启动扫码；开局提前收到的扫码结果会由调度暂存。下一轮须重新启动扫码节点。

## 复位与现有手动启动方式

仍使用已有 `/mission/pause`、`/mission/reset`。比赛结束、暂停或故障后，人工恢复车辆、物料与机构，再按原有规则复位；屏幕控制模式复位后回到 `WAIT_SETUP`，重新选择本轮路径及起终点。不自动重跑失败的比赛。

原 `ros2 launch agv_mission mission.launch.py` 保留 YAML 起终点和手动 start 方式，默认 `screen_controlled: false`；新联合入口覆盖为 true。屏幕控制模式下不能绕过配置在 `WAIT_SETUP` 直接 start。

## 第四阶段待补

`screen_node.py` 的 `FINISHED` 分支标有 `TODO(stage4)`。后续视觉节点提供可确认的抓取/放置结果后，应由 mission 按本轮身份统计、去重，发布最终成功抓取数和成功放置数；屏幕节点收到后发送 `page result`，等待至少 300 ms，再赋值 t4、t5。确认统计口径后实现，不用总步骤数代替两个成功数，也不显示占位 0。

## 验证

测试不接实物；使用 PTY 串口、真实 ROS2 节点与服务，并将底盘输出重映射到测试话题。在独立 domain 中运行，避免与实车节点交互：

```bash
source /opt/ros/humble/setup.bash
source /root/工创/ros2_ws/install/setup.bash
ROS_DOMAIN_ID=87 python3 -B -m unittest discover -s /root/工创/ros2_ws/src/agv_screen/test -v
```

屏幕串口读写错误会停止通信节点并记录错误，不自动重启比赛。调试原始串口消息可给节点设置 ROS 日志级别 debug。正式比赛不运行 `test/` 中的终端交互脚本。
