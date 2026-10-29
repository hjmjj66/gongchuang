# 比赛调度

Python ROS2 Humble节点 `mission_node` 管理启停区斜移、开局扫描及十五比赛阶段。区域任务接收整批数组；调度统一转发底盘速度，不计算导航速度，不重复发机构命令。暂停后必须复位，复位只清软件状态，车辆、物料及机构由人工恢复。

屏幕正式接入见 [agv_screen](../agv_screen/README.md)。联合入口 `ros2 launch agv_screen screen_mission.launch.py` 启动调度和屏幕通信，覆盖 `screen_controlled:=true`：先等待屏幕本轮完整配置，再初始化起点，接受 OK 至少两秒且 READY 后自动开始。通过 `/mission/configure_and_start`（`ConfigureMission`）提交当前 `run_id`、start/stop（1→节点1，2→节点3）和人工禁行编号。雷达扫描结果与人工禁行边取并集后，由 mission 统一提交地图；复位后清除人工配置并重新等待屏幕确认。原入口默认保持 YAML 起终点与手动 start。

第四阶段 TODO：真实视觉确认结果和统计口径补齐后，由调度生成本轮成功抓取/放置统计，再交给屏幕显示；当前不从总步骤数推定成功数，不发送占位数字。

## 导航与速度出口

mission通过 `/route/plan` 发目标节点及是否作业对齐，继续使用 `start=0`，起点、当前朝向和配置的终点作业朝向由路径节点确定。路径节点读取 `/C/odometry`，完成路线规划、动作合并、减速和到达判断。

比赛起点与最终停车终点统一配置在 `agv_mission/config/mission.yaml`：

```yaml
start_area: left   # 比赛起点；面向二维码板时，左侧节点1为left，右侧节点3为right
finish_area: left  # 比赛最终停车区；left / right，与起点独立配置
parking_offset_mm: 200  # 出入启停区时两条车体轴各自的位移，正整数mm
```

同区起停时两项填相同值，也可配置从1出发最终停到3，或反向。mission初始化/复位时通过 `/route/reset_context` 把启动节点及朝向基准同步给路径节点，开局车头始终朝二维码区2；后续每次导航仍只发目标，路径节点维护当前位置和朝向。软件复位仍要求人工将车恢复到 `start_area`。比赛不需要再修改planner YAML中的起点。

比赛开始后先保持车头方向、两轴同时斜移：从1出发前进并右移，从3出发前进并左移，各走 `parking_offset_mm`。到达1的左下角或3的左上角后才开始雷达扫描，再沿正常路线到二维码区；角点到二维码区仍走完整一条配置边长，不加减斜移距离。初始摆车和C板里程原点均在启停区内，路径边长从出区后的角点计算。

最后先按正常路线抵达 `finish_area` 对应角点，再保持当时朝向斜移入区，两轴各走同一个参数值；两轴到位、路线完成状态确认后才发布 `FINISHED`。参数修改后重启mission。入区动作按地图朝向换算如下（距离均为每轴位移）：

| 车头朝向 | 进入1（地图右上方） | 进入3（地图右下方） |
| --- | --- | --- |
| UP | 前进、右移 | 后退、右移 |
| RIGHT | 前进、左移 | 前进、右移 |
| DOWN | 后退、左移 | 前进、左移 |
| LEFT | 后退、右移 | 后退、左移 |

外部执行端已有阶段编号保持不变；新增 `EXIT_START_AREA=16`、`ENTER_FINISH_AREA=17`。实际顺序为16→0（扫描）→1…15（返回角点）→17（入区）→FINISHED。两个斜移阶段继续使用route执行器和现有暂停、故障、里程及速度反馈约束。

```text
mission --目标节点--> 路径节点
mission <-- /route/control：速度、执行状态、任务身份 -- 路径节点
mission -- /C/controll：ControlCommand --> C板通信 --> C板
C板通信 -- /C/odometry --> 路径节点
```

`/route/control`类型为 `agv_interfaces/msg/RouteControl`，包含 `command`（vx_mps、vy_mps、omega_deg_s）、`state`（EXECUTING / COMPLETED / ABORTED）、时间戳、任务/本轮/进程身份及递增序号。只有已接受的当前导航任务、匹配来源且新鲜的反馈才转发；旧轮、旧任务、重复、乱序及过期反馈不会更新速度有效期。`/route/execution_status`仍保留。

完成或中止时mission先清除旧速度并归零；成功还需查询确认对应任务、终点和朝向，才进入下一阶段。暂停、故障、复位以及待命时也保持零速。当前有效导航反馈中断超过 `route_control_timeout_s`（默认0.3秒）即故障停车，不重放缓存中的非零速度。C板通信只订阅mission维护的 `/C/controll`，不得再订阅路径原始反馈。以后视觉伺服速度也应通过mission按活动阶段转发，尚未实现的区域视觉节点不能直接竞争最终话题。

新配置项为 `route_control_topic / control_topic / route_control_timeout_s / control_rate_hz`，中文说明见 `config/mission.yaml`。最终速度字段为vx向前正、vy向左正（m/s），逆时针正（°/s）。旧版配置和消息需重新构建、重启。

## 构建与比赛运行

```bash
source /opt/ros/humble/setup.bash
cd /root/工创/ros2_ws
colcon build --symlink-install --packages-up-to agv_mission
source /root/工创/ros2_ws/install/setup.bash
ros2 launch agv_mission mission.launch.py
```

另一个终端 source 后调用：

```bash
ros2 service call /mission/start std_srvs/srv/Trigger '{}'
ros2 service call /mission/pause std_srvs/srv/Trigger '{}'
ros2 service call /mission/reset std_srvs/srv/Trigger '{}'
ros2 topic echo /mission/state
ros2 topic echo /mission/task_info
```

等状态READY再开始。pause先返回接受，待当前完整机构程序结束才PAUSED；reset后等READY再start，不提供resume。所有状态转换、阶段及物料确认打印日志。

## 真实模块接入

```bash
ros2 launch agv_mission mission.launch.py mode:=competition start_area:=left
```

此入口只启动调度，不启动尚未实现的硬件业务。缺少执行端、工位朝向或里程反馈时拒绝开始/初始化失败，不产生模拟成功。

默认配置为 `competition`，起点和终点均为 `left`。使用 `config:=/绝对路径/mission.yaml` 时读取该文件中的模式、起点和最终停车区；只有显式传入 `mode:=...`、`start_area:=...` 或 `finish_area:=...` 才覆盖对应配置，修改起点不会覆盖YAML中独立设置的终点。

每个执行端需要 `get_runtime / stop / reset_context` 服务，类型在 `agv_interfaces`。路线前缀 `/route`，雷达 `/obstacles`，主视觉 `/vision`。区域Action为 `/vision/raw/run_batch`、`/vision/coarse/run_batch`、`/vision/storage/run_batch`，雷达为 `/obstacles/scan`。

真实路线节点已实现运行查询、停止、身份绑定、地图隔离、跨轮复位和调度心跳超时停车。独立调试使用空run_id；执行比赛复位后仅接受相应身份。调度使用 `/mission/heartbeat` 发布当前run_id。

比赛使用 `competition`，强制所有执行端来源为REAL。真实执行器重启、任务未知或结果不完整时进入FAULT，不自动重试或续跑。接口保留来源隔离校验，项目不再提供模拟执行节点或模拟启动入口。

真实扫码连续 5 次得到相同文本后向 `/qr/result` 发布一次并释放相机。先启动调度器并建立订阅，再启动扫码节点；调度暂存本轮提前收到的首个有效任务码，进入扫码阶段后保存快照并向 `/mission/qr_result`（`std_msgs/msg/String`）转发一次。输入/输出话题由 `qr_input_topic` / `qr_result_topic` 参数配置。原始输入保持 VOLATILE，转发话题为 RELIABLE、TRANSIENT_LOCAL、depth=1，晚加入的订阅者需使用匹配 QoS。复位清除本轮任务及暂存，下一轮须重新启动扫码节点；转发缓存仍表示最近一次已接受结果，当前轮有效性以 `/mission/task_info` 的 `run_id`、`valid` 为准。

真实主相机区域业务、雷达识别算法、C板桥接和二维局部返回仍由相应模块实现；本包提供调度和完整接口。控制器动作完成记录不冒充传感器证实抓到物料。开发测试、临时检查文件及模拟入口已按实车目录清理，运行依赖不齐时保持拒绝开始。

## 执行端实现要点

- `get_runtime`返回真实进程身份和完整空闲状态；REAL与MOCK来源不在比赛中切换。
- `reset_context`需要预期旧run_id、新run_id及幂等reset_id；只在已停止时接受。执行端重启后更换session_id。
- RunWorkBatch反馈中的confirmed_steps是按顺序排列的累计前缀，Result包含完整列表；步骤ID使用 `PICK:1 / PLACE:1 / STACK:1` 等，以任务身份区分进站。
- Action取消时禁止提交新机构动作，已提交程序完成后才返回终态；取消接受本身不代表物理动作已经停止。
- 区域成功必须同时确认整批动作、离站参考、底盘释放和机构空闲；首批暂存还返回颜色位置确认表。
- 软件停止确认使用暂停后发起的运行查询，未决请求未明确前不会宣布PAUSED或接受复位。
- `/route`当前静止标识仅代表软件速度循环停止；C板物理停车反馈仍需硬件集成补齐。

## 二维里程与控制恢复

最终速度统一车体系 x前/y左、逆时针为正。REAL 模式监视 `/C/status` 的新鲜度、控制权和坐标/许可变化；活动比赛中异常立即停止转发并终止旧上下文，包括区域等待期间。MOCK 路线不依赖真实 C 板。

自动路线的 `ControlCommand.odom_session_id/motion_epoch` 必须匹配当前 C 板状态，并原样转发。通信恢复不恢复旧任务。正式比赛先在起点摆好车、切PC建立里程原点，再初始化和开始。位置进度来自规划器的 x/y 方向投影，不再读取实测累计路程。
