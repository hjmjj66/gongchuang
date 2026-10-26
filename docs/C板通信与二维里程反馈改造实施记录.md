# C 板通信与二维里程反馈改造实施记录

日期：2026-10-25。对应需求：[改造计划](C板通信与二维里程反馈改造计划.md)。软件改造已完成，已在 ROS2 Humble 下构建并通过核心、ROS/PTY 和浏览器链路验证；没有连接或驱动实车，没有修改下位机固件。

## 已实施的行为

- `/C/controll` 保持上位机速度输入；通信节点不向自身输入话题发布零值。串口持续 50 Hz 发包，空闲或不允许运动时发全零、mode=0。
- `command_timeout_s: 0.05` 写入 `agv_cboard/config/cboard.yaml`；同时修改代码默认值。ROS 输入源时间超时、乱序和恢复门槛检查继续生效。50 Hz 周期发送下，零速下发发生在超时后的下一发送周期。
- 控制指令和实际车体系速度统一前 x / 左 y，逆时针为正；控制输入 omega 单位 deg/s，串口和实际 wz 单位 rad/s。
- `/C/odometry` 已移除实测累计里程，提供要求的 x/y、两种 yaw 单位和三轴实际速度，增加接收时间、板时间、健康/初始化标志、坐标会话和运动许可代次。
- 每次非 PC→PC、桥接进程启动、串口重连或确认板端重启，都重建原点；用下一条健康新 ODOM 建立位置与航向零点，并旋转位置轴。连续 PC 状态不重复归零。
- `/C/chassis_odometry` 保留板端原始值。两种观测与运动许可分离，RC 下继续发布新采样；无新帧或断连不重放旧值，异常观测标为无效。
- 健康恢复后自动接收新速度；旧指令清除，`fault_reason` 更新为当前原因，不再锁存旧非 PC 原因。兼容的 `/C/reset_fault` 只查询当前健康，不是恢复前提。
- 自动任务绑定 `odom_session_id/motion_epoch`，即使旧任务的指令发布时间很新，通信节点仍拒绝旧绑定，避免恢复与异步任务中止之间续跑。
- `/C/status` 还提供输入年龄、指令拒绝原因，以及最近成功写入串口的速度/mode、序号和年龄。串口写出不代表板端已经执行，实际速度仍取 ODOM。

## 规划器与调度

`agv_planning` 从段起点和当前 x/y 计算沿本段固定目标方向的有符号投影，保留原有路线生成、动作合并、减速和到达门槛。横向、反向和原地旋转不作为累计前进。段方向按开始时车身航向确定，不随每帧 yaw 改变；横向偏差只用于显示，本轮没有横向纠偏或锁头闭环。

移除了反馈累计路程、累计值回退故障和旧段距离基线。`RouteExecutionProgress` 改为起点、当前点、目标方向、纵向投影和横向偏差；平移进度单位 mm、转向进度单位 deg。完成后保留实测段基线，不用目标值覆盖实际纵向/横向结果。

`RoutePlan.total_distance_mm` 和拓扑规划结果中的同名字段仍表示计划路线总长度，保留；逐边长度、目标距离、减速区也保留 mm 单位。它们不是旧实测累计里程依赖。

控制权/里程健康/会话/许可变化会终止旧路线并撤销地图位置绑定；新局部坐标不等于知道比赛地图位置。重新确认位置与朝向并初始化执行上下文后才允许新任务。

正式 mission 的 REAL 路线监视 C 板状态，并原样校验转发指令绑定；包括工作等待等非导航阶段的状态变化也会阻止继续旧比赛。MOCK 路线继续与真实底盘隔离，不强制依赖真实 C 板。用户的规划速度、减速区、容差和边长数值没有被改成另一套默认值。

## 调试工具

- 页面从实测 x/y 和人工确认的节点/朝向建立地图锚定，绘制实际里程轨迹；不再把车贴在规划道路上，完成时不强行吸附终点。
- 输入指令和实际速度分别显示，标签统一前/左/转；显示固定系 x/y、连续 yaw、坐标会话、许可代次和当前段进度/横向偏差。
- 切 RC、原点/许可变化终止本轮，撤销地图锚定、清除旧轨迹和等待。停止结果确认后重新选择当前位置、朝向，直接点击开始即可；不要求先调用清锁服务或先点一次复位。
- 原有单发布者检查、浏览器失联停车、重复命令去重、未知 RPC 结果确认仍保留。
- 自由行走和模拟比赛保持现有语义，后者仍为 8 条路线和 7 次三秒等待。

显示采用现有截图像素锚点与平行边平均米制标定；平行边不构成一致矩形时页面提示近似。该转换只用于画图，不参与导航进度，不消除打滑或航向漂移。独立实测地图节点标定配置作为后续显示增强，本轮使用可提示误差的现有网格标定。

## 诊断工具

新增 `ros2 run agv_cboard cboard_diagnostics`：

```bash
# 只读记录；车辆由用户确认保持静止
ros2 run agv_cboard cboard_diagnostics monitor --duration 60 --output /tmp/cboard_static.csv

# 主动运行此命令会让车低速前进；先结束其他最终速度发布者
ros2 run agv_cboard cboard_diagnostics pulse --duration 3 --vx 0.05 --output /tmp/cboard_pulse.csv
```

CSV 包含原始/会话观测、当前状态；pulse 额外记录实际发布的命令。monitor 不创建速度发布者，输出原始 yaw 斜率、平均 wz 和积分。pulse 等待订阅匹配与健康 PC 反馈，从第一条非零实际发布起计时，期间状态/会话变化或竞争发布者出现会中止，最后主动发零。

输出文件独占创建，重复测量请换文件名。前进使用 vx，左移使用 vy。CLI 的进程存活三秒不保证实际指令发满三秒。

静止 yaw 问题：目前上位机没有航向积分或漂移补偿，两个底盘观测都来自下位机同一 yaw；`/C/status.yaw_rad` 是机构轴。已有现象指向板端上报漂移，但未取得本轮实车静止数据和下位机源码，尚不能定责到某个 IMU/融合实现；需采样后将漂移率、原始 wz 和状态交电控检查。上位机重置只改变零点，不能消除随时间的漂移。

## 自动化验证

2026-10-25 最终执行完整回归入口，47 个 Python 用例和 1 组浏览器脚本全部通过；其中 C 板 ROS/PTY 为 3 例，网页完整链路为 2 例。另检查了 38 个 Python 文件的语法、测试/启动 shell 语法及计划与实施记录中的本地链接。测试中出现的控制权、断连、无进展等错误日志来自刻意注入的异常场景，对应恢复/终止断言均通过。

| 验证 | 用例数 | 结果与覆盖 |
| --- | --- | --- |
| C 板协议/核心 | 19 | CRC/流解析、左右前轴、50ms超时、源时间、RC/PC、自动恢复、原点、重连、时钟回绕/重启 |
| 调试 session/地图 | 12 | 8段/7等待、终态先到、停止与未知结果、旧消息、会话重置后开始、实测轨迹不吸附 |
| 二维规划执行 | 6 | 多航向投影、反向/横向、无进展、无效反馈、坐标/许可失效及上下文重建 |
| mission 底盘契约 | 5 | 绑定转发、旧代次拒绝、非导航阶段重置、状态超时、MOCK隔离 |
| C 板 ROS/PTY | 3 | installed launch、实际串口周期、自动恢复/断连、真实计时pulse、只读采样不发运动 |
| 网页/规划/桥接/PTY | 2 | HTTP/SSE、完整比赛、停止/复位、同点转向、RC→PC终止与新开始、反馈与浏览器失联；正式复位边界 |
| 浏览器脚本 | 1组 | 模式、设置、状态和错误显示、失联、按钮可用性 |

三秒 pulse 集成用例检查了伪串口非零包持续时长与前向编码，并验证结束后归零；不是通过观察进程存活时间推断。所有运动集成测试只打开新创建的 PTY。

ROS/DDS 与 HTTP 的套接字验证需要本机网络权限，沙箱受限时在批准的本机测试环境运行；不访问外网，不连接实车。

完整自动回归入口为 `/root/工创/tools/route_debug/tests/run_migration_checks.sh`（仅伪串口）。也可分项运行（先 source ROS 与 workspace，ROS 测试放独立 domain）：

```bash
PYTHONPATH=/root/工创/ros2_ws/src/agv_cboard python3 -m unittest discover -s /root/工创/ros2_ws/src/agv_cboard/test -p test_core.py -v
PYTHONPATH=/root/工创/tools/route_debug python3 -m unittest discover -s /root/工创/tools/route_debug/tests -p test_session.py -v
node /root/工创/tools/route_debug/tests/test_web.js
python3 -m unittest discover -s /root/工创/ros2_ws/src/agv_planning/test -p test_xy_execution.py -v
python3 -m unittest discover -s /root/工创/ros2_ws/src/agv_mission/test -p test_chassis_contract.py -v
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=93 AGV_CBOARD_ROS_TEST=1 python3 -m unittest discover -s /root/工创/ros2_ws/src/agv_cboard/test -p test_ros_pty.py -v
/root/工创/tools/route_debug/tests/run_ros_pty.sh
```

## 使用新版本

旧消息不兼容。结束旧控制节点后，统一构建并在各新终端重新加载环境：

```bash
source /opt/ros/humble/setup.bash
cd /root/工创/ros2_ws
colcon build --symlink-install --packages-select agv_interfaces agv_cboard agv_planning agv_mission agv_perception
source /root/工创/ros2_ws/install/setup.bash
ros2 launch agv_cboard cboard.launch.py serial_port:=/dev/ttyACM0
```

单独启动桥接即可观察通信；调试运行 `/root/工创/tools/route_debug/start.sh serial_port:=/dev/ttyACM0` 时由工具统一启动桥接/规划器，不要再开另一份桥接或 mission。

## 尚需实车验收与协议限制

实车尚未执行：四方向与转向、不同朝向重建坐标、三秒前进的实际速度/位移、切 RC 后重新开始、实物重连/重启、静止 yaw 漂移及地面停车偏差。A 方案的投影到达不保证二维位置精确到达；横向纠偏与航向闭环仍属于后续 B 方案。

板端重启通过 STATUS/ODOM 两条时间流共同回退确认，uint32 自然回绕不当成重启，单条乱序不直接重建坐标。现有协议没有启动身份，无法保证检测“时间/坐标恰好看似连续”的所有快速重启；若需要无歧义识别，应由电控增加明确启动标识并另行约定协议。
