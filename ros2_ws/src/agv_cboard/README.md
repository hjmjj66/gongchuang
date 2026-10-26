# C 板底盘通信节点

`cboard_bridge_node` 订阅上位机最终速度 `/C/controll`，按 50 Hz 经 USB CDC 发给 C 板；发布会话二维里程 `/C/odometry`、板端原始观测 `/C/chassis_odometry` 和当前状态 `/C/status`。

## 构建与启动

ROS2 Humble，依赖 `python3-serial`。本次消息不兼容旧版本，结束旧节点后统一构建：

```bash
source /opt/ros/humble/setup.bash
cd /root/工创/ros2_ws
colcon build --symlink-install --packages-select agv_interfaces agv_planning agv_cboard agv_mission agv_perception
source /root/工创/ros2_ws/install/setup.bash
ros2 launch agv_cboard cboard.launch.py serial_port:=/dev/ttyACM0
```

默认使用 YAML 中的 `/dev/rm_cboard`；设备链接规则见 `udev/99-rm-cboard.rules`。`config:=/绝对路径/cboard.yaml` 可覆盖配置；只有明确设置 `serial_port:=...` 才覆盖 YAML。参数在启动时读取。

## 坐标与速度

所有输入和实际车体系速度统一：**x 向前、y 向左、yaw 逆时针为正**。`vx_mps` 是当前车头前向速度，`vy_mps` 是当前车身左向速度；输入角速度 `omega_deg_s` 单位 °/s，串口和反馈 `wz_rad_s` 单位 rad/s。

`/C/odometry` 的位置属于固定会话坐标系。每次进入 PC、桥接重启、串口重新连接或确认板端重启，都以之后第一条健康 ODOM 建立 `(0,0,0)`：+x 为该时刻车头前方、+y 为该时刻车身左侧。车辆之后转向，位置轴不随车旋转。初始化之前 `origin_initialized=false`、`valid=false`。

原始样本 `(xb,yb,yawb)`、原点 `(x0,y0,yaw0)` 的变换：

```text
x = cos(yaw0)*(xb-x0) + sin(yaw0)*(yb-y0)
y = -sin(yaw0)*(xb-x0) + cos(yaw0)*(yb-y0)
yaw = yawb-yaw0
```

实际速度保持当前车体系，不随原点变换旋转。航向连续展开、不回绕；单位换算不补偿漂移。

## 接口

| 接口 | 类型与内容 |
| --- | --- |
| `/C/controll` | 输入 `ControlCommand`：前 vx、左 vy、逆时针 omega；通信节点不向这个输入话题补发零速 |
| `/C/odometry` | `OdometryFeedback`：`x_m,y_m,yaw_rad,yaw_deg,vx_mps,vy_mps,wz_rad_s`；采样时间、板时间、状态、有效性、坐标会话与许可代次 |
| `/C/chassis_odometry` | `CBoardOdometry`：不减零点、不旋转的板端原始位置、连续航向、实际速度、健康位 |
| `/C/status` | `CBoardStatus`：当前连接、控制权、里程有效性、运动许可、当前故障原因、指令新鲜度及串口实际写出信息 |
| `/C/reset_fault` | `Trigger`：兼容健康查询，自动恢复无需调用；不会重置坐标或恢复任务 |

两种里程均只发布新收到的可解析采样，最多 100 Hz；RC 下继续观测，按下述打滑策略判断有效性，其他健康异常标为无效。断链或无新帧不重放旧值。时间戳近似本地接收时间，未与板端时钟同步。没有额外生成 TF。

`odom_session_id` 在重建原点时改变；`motion_epoch` 在运动许可失效时改变。自动任务的 `ControlCommand` 必须携带这两个绑定，旧绑定即使刚发布也拒绝执行。手动 CLI 可只填写三个速度字段，但仍检查健康和时序。

## 超时与恢复

| 参数 | 默认值 |
| --- | --- |
| `send_rate_hz` | 50.0 |
| `command_timeout_s` | **0.05** |
| `status_timeout_s` | 0.2 |
| `odometry_timeout_s` | 0.2 |
| `reconnect_interval_s` | 1.0 |
| `stop_on_slip` | 节点默认 `true`，项目 `config/cboard.yaml` 配置为 `false` |

无新鲜输入、非 PC、轮/IMU/编码器异常、反馈过期、原点未初始化或旧任务绑定时，串口持续发全零、`mode=0`；满足条件才发送 `mode=1`。50 Hz 串口周期下，最后输入超过 50 ms 后在下一次发送周期归零，正常调度下约为输入后 50～70 ms。

`stop_on_slip: false` 时，单独上报 ODOM bit1（打滑，通常 `odom_status=0x0F`）只记告警，不改变运动许可代次、不使路线中止；两种里程的 `valid` 与状态的 `odometry_valid` 均按同一策略判断，仍要求 valid/IMU/编码器三位正常（`flags & 0x0D == 0x0D`）。原始 `odom_status` 保留打滑位，继续运行时里程可能有打滑误差。设为 `true` 则恢复打滑即停车。此参数在启动时读取，修改后重启桥接。PC 桥接没有低速或 2 m/s 门限；板端打滑判定比较轮解算与陀螺角速度，调整该门限需要 C 板固件。

使用 Reliable/Volatile/KeepLast(1) 和 `CommandExecutor` 保留 DDS 源时间，防止积压旧速度续命；不要嵌入丢弃元数据的普通 `rclpy.spin()`。输入源时间必须有效；跨电脑发布须同步系统时钟。

反馈健康、PC 权限恢复后自动接收新的速度，不需人工清锁，也不重放旧速度。`fault_reason` 只显示当前健康原因；普通待命、输入超时和旧指令拒绝原因见 `command_reason`。历史变化保留在日志中。

`tx_sequence/tx_age_s/sent_vx_mps/sent_vy_mps/sent_wz_rad_s/sent_mode` 是最近成功写入串口的帧，不是下位机已经执行的保证。板端实际速度在两个里程话题中。`motion_allowed` 与 `command_fresh` 分离，均不等于物理停稳。

自动任务遇到坐标或许可变化会终止；通信恢复不能自动恢复旧路线。调试重新选择当前位置、朝向后点击开始。正式比赛先在配置起点摆好车、切 PC，再初始化比赛上下文。

## 静止漂移采样与三秒测试

结束其他最终速度发布者，source 环境。只读采样（车辆保持静止）：

```bash
ros2 run agv_cboard cboard_diagnostics monitor --duration 60
```

默认直接运行，在终端输出原始 yaw 斜率、平均 wz 及积分；实时里程、实际速度和状态可在 Foxglove 观察。`/C/status.yaw_rad` 是机构轴，不是底盘 yaw。若原始底盘 yaw 在物理静止时仍漂移，可将终端统计值交电控检查零偏、标定与融合；本节点不人为减漂移。需要额外 CSV 采样时才添加 `--output 路径`，文件使用独占创建；省略该参数不会创建采样记录文件。

显式前进三秒（会运动，须在现场主动运行；`--vx`、`--vy` 和 `--omega` 按命令值发送，不受诊断 CLI 的人为速度上限限制）：

```bash
ros2 run agv_cboard cboard_diagnostics pulse --duration 3 --vx 0.05
```

工具等待订阅连接、PC 和健康反馈就绪后，才从实际发布开始计时，50 Hz 发满三秒并主动发零。其他发布者、反馈或会话变化时中止。三个速度参数默认均为零：向左使用 `--vy 0.05`，原地逆时针转动使用 `--omega 10`；只有显式同时指定两个非零轴才会合成运动。旧版 `--vy` 默认 `0.05`，只指定 `--vx` 或 `--omega` 会附带非目标 vy 轴速度；升级后需重新构建并加载 workspace。不要用进程启动总时长代表实际发指令时长；0.15 m 只是理想速度乘时间，不包含加减速与低速响应误差。

## 固定距离与角度测试

按里程反馈执行单轴相对距离或相对角度（会运动，完整可复制步骤见根目录《C板实车测试操作指南》第 6 节）：

```bash
# 本次开始时的前向 0.20 m；负数表示向后。
ros2 run agv_cboard cboard_diagnostics move --dx 0.20 --speed 0.05 --timeout 30
# 本次开始时的左向 0.20 m；负数表示向右。
ros2 run agv_cboard cboard_diagnostics move --dy 0.20 --speed 0.05 --timeout 30
# 相对当前航向逆时针转 90°；负角度表示顺时针，omega 填正数上限。
ros2 run agv_cboard cboard_diagnostics move --angle 90 --omega 10 --timeout 30
```

一次只运行一条。`move` 使用本次起步车体系方向上的有符号位置投影或连续 yaw 差值，接近终点减速，默认剩余 5 mm / 1° 时发零；未达到目标方向进展 3 秒或运动超过 `--timeout` 则报错停车。坐标/许可变化、无效或过期里程和竞争发布者也会中止。该功能不做横向纠偏、航向保持或过冲反向校正，不保证地面实际停车精度；停止后持续发零 0.5 秒并报告最终观测误差。

## 自动验证

```bash
PYTHONPATH=/root/工创/ros2_ws/src/agv_cboard python3 -m unittest discover -s /root/工创/ros2_ws/src/agv_cboard/test -p test_core.py -v
PYTHONPATH=/root/工创/ros2_ws/src/agv_cboard python3 -m unittest discover -s /root/工创/ros2_ws/src/agv_cboard/test -p test_relative_move.py -v
```

ROS2 与 workspace 已 source、使用独立 domain 后，`AGV_CBOARD_ROS_TEST=1` 可启用 `test_ros_pty.py`；仅打开新建 PTY，不连接实物。完整网页链路见 `tools/route_debug/tests/run_ros_pty.sh`。自动验证与实车验收分别记录。
