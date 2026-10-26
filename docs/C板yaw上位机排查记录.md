# C 板 yaw 上位机排查记录

## 结论及适用范围

当前源码及工作空间内已安装的桥接代码中，未发现会让固定输入 yaw 随时间漂移的逻辑。使用真实 ROS 节点、虚拟串口及 ROS 订阅者验证，固定板端 yaw 时发布的 yaw 严格不变；非零角速度、位置变化、时间推进和运动命令不会改变此结论。串口停止上报后不会外推或重复发布里程。

这证明当前上位机链路不会自行制造本次怀疑的持续 yaw 漂移，不代表车辆静止时 yaw 漂移正常，也不能替代故障发生时对实际发布者和输入帧的核对。本轮仅补充回归测试和记录，未修改生产逻辑，未操作真实底盘。

## 代码链路核对

| 环节 | 核对结果 |
| --- | --- |
| 串口解析 | 按帧头、长度及 CRC 解析；不会把发出的速度指令作为 ODOM 接收 |
| ODOM 布局 | 小端；yaw 在载荷偏移 12–15，wz 在 24–27，与协议一致 |
| 机构角与底盘角 | STATUS 的机构 yaw 与 ODOM 的底盘 yaw 分开存储，不会互相覆盖 |
| 原点建立 | 建立后保存一份原始 yaw0；普通 ODOM、健康恢复及位置变化不会更新它 |
| 会话 yaw | `local_yaw = raw_yaw - yaw0`；无积分、滤波、补偿或角度回绕 |
| 单位 | `yaw_deg = degrees(yaw_rad)`，仅换算一次 |
| ROS 发布 | 新 ODOM 产生一次待发布样本，发布后清除；断流不继续积分或重播 |
| 会话变化 | 进入 PC、串口重连、确认板端重启会重建原点；会改变 session，不会形成同一会话内的平滑漂移 |
| 规划与页面 | 消费反馈并换算显示，不发布 `/C/odometry`，不回写底盘 yaw |
| 安装代码 | `protocol.py`、`transport.py`、`chassis_adapter.py`、`cboard_bridge_node.py` 与源码逐字一致 |

关键文件：

- `ros2_ws/src/agv_cboard/agv_cboard/protocol.py`
- `ros2_ws/src/agv_cboard/agv_cboard/chassis_adapter.py`
- `ros2_ws/src/agv_cboard/agv_cboard/cboard_bridge_node.py`
- `ros2_ws/src/agv_cboard/agv_cboard/transport.py`

## 实际验证

`test_core.py` 共 27 项通过。其中新增：

1. 独立按协议字节偏移构造 ODOM，各字段填入不同数值，验证 yaw/wz 未读错；向 STATUS 写入另一个机构角，底盘 yaw 保持原值。
2. 连续 100 次输入固定 yaw、变化位置、非零反馈角速度及转向命令，然后持续调用超时检查，yaw 始终不变。

真实 ROS + PTY 集成测试共 3 项通过，使用独立域 `ROS_DOMAIN_ID=97`、`ROS_LOCALHOST_ONLY=1`，仅打开虚拟串口。

新增 `test_yaw_pipeline_fixed_input_changes_sessions_and_silence` 验证：

- 原始 yaw 固定为 1.25 rad、反馈 wz 为 -0.5 rad/s，持续接收时局部 yaw 精确保持 0。
- 改变 x/y 及 wz，yaw 和会话不变。
- 输入 -0.625、-7.25、13 rad，原始输出保留输入，局部输出精确等于输入减 1.25；超过一圈时不回绕。
- 按 `board_time_ms` 匹配同一原始/局部样本，验证 `local_yaw = raw_yaw - yaw0`。
- PC→RC 时保持原会话；RC→PC 后建立新会话，局部 yaw 归零。
- 停止 ODOM、保持 STATUS 和发送线程运行，超时后健康变无效，原始/局部里程消息计数均停止增长。
- 未创建运动命令发布者时，桥接发出的串口命令始终为零。

另外两项已有集成测试验证了断线、重连、待发命令超时、DDS 阻塞和两种打滑处理策略。

复现新增集成测试：

```bash
source /opt/ros/humble/setup.bash
source /root/工创/ros2_ws/install/setup.bash
export ROS_LOG_DIR=/tmp/cboard_yaw_pty_roslog
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=97
export AGV_CBOARD_ROS_TEST=1
export PYTHONPATH=/root/工创/ros2_ws/src/agv_cboard:/root/工创/ros2_ws/src/agv_cboard/test:$PYTHONPATH
/usr/bin/python3 -m unittest test_ros_pty.RosPtyTests.test_yaw_pipeline_fixed_input_changes_sessions_and_silence -v
```

## 与之前被动串口观测的关系

之前在桥接节点未运行时直接只读采集 STM32 串口，排除开头 0.5 秒缓存后，5 秒收到 501 条有效 ODOM；板端时钟精确前进 5 秒，帧间隔均为 10 ms。x/y 不变、vx/vy 为零时，原始 yaw 净变化约 +0.02284°。这说明当时变化已存在于协议约定的原始 yaw 字段中，上位机原点变换和 ROS 发布没有参与产生这段变化。

这段采样没有确认物理静止及实际供电接法，因此不能直接认定具体 IMU 算法故障，也不能证明 USB 是实际供电路径。IMU/编码器健康位不代表陀螺零偏为零；反馈速度为零也不保证融合角严格恒定。

若要进一步定位实车现象，应在同一会话、按同一板时间对齐原始和局部消息：差值应恒定为 yaw0。若原始 yaw 不变而局部 yaw 漂移，才与本轮验证结果矛盾，需要检查现场发布者、运行代码和显示数据源。若两者等量变化，则变化来自串口上报字段，下一步需要核对板端打包及姿态计算实现。
