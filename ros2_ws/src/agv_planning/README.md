# 3×3 路径规划与里程计执行节点

面向 ROS2 Humble，包 `agv_planning`，可执行文件 `route_planner_node`。
节点等待地图、二维里程和规划请求，成功规划后自动执行。路线与离散行动链仅作调试查看，实际运动通过速度话题完成。

浏览器实车调试入口见 [tools/route_debug](../../../tools/route_debug/README.md)。工具启动时单独开启 `debug_allow_any_pose`（允许1～9任意初始节点和四个方向）与 `debug_publish_progress`（10 Hz `/route/progress`，`RouteExecutionProgress`，工具内映射到私有话题）。两项正式默认均为false，保留比赛启停区复位限制；进度只供显示，不参与控制。

路径节点作为mission调用的导航工具：mission只指定目标节点及是否作业对齐，起点和当前朝向仍由路径节点内部确定。路径节点向 `/route/control` 返回速度和执行状态，由mission统一转发速度到 `/C/controll`；C板通信只订阅mission的最终输出。

比赛接入新增 `/route/get_runtime`、`/route/stop`、`/route/reset_context`，分别为 `GetExecutorRuntime`、`StopExecutor`、`ResetExecutor` 服务，类型位于 `agv_interfaces`。运行查询包括当前任务、最近终态、进程身份、节点/朝向、里程健康、地图版本及封锁边。

`PlanRoute`、`RoutePlan`、`RouteExecutionStatus`、`BlockedEdges` 新增 `run_id`。未绑定比赛时可省略以保持独立调试用法；比赛初始化后仅接受匹配身份，比赛路线使用 `start=0` 和反馈朝向。地图也必须携带本轮身份。旧程序须重新构建以匹配更新后的消息。

停止服务按活动身份停止速度控制循环并归零，中途停止后位置未知。复位使用 `expected_run_id / run_id / reset_id` 实现比较并交换及幂等性，仅在空闲时接受，清地图与旧里程检查基线，并等待下一条新里程建立yaw基准。复位代表人工已恢复初始现场，不自动返航。

比赛的 `start_area / finish_area` 统一放在 `agv_mission/config/mission.yaml`。mission初始化/复位时同步起点，最终停车区作为最后一次导航的明确目标发送；planner YAML不再重复配置起点。单独调试规划器且未绑定mission时，仍默认从节点1启动，可用节点参数 `start_area:=right` 改为节点3；这项调试默认值会在比赛初始化时被mission覆盖。

正式比赛中，拓扑节点1表示启停区1左下角，节点3表示启停区3左上角。mission先请求原地节点的 `parking_maneuver: EXIT` 斜移出区，再扫描、提交地图并请求正常路线；最后抵达目标角点后请求 `ENTER` 斜移入区。`PlanRoute.parking_offset_mm` 表示两条车体轴各自的位移，由mission的同名参数提供，默认200 mm；合成斜线约283 mm。两个动作均保持车头朝向，按反馈朝向自动选择前后及左右。正常拓扑边长从角点起算，仍完整使用 `default_edge_length_mm / edge_length_mm`。

斜移请求要求起终点相同且为1或3，不允许作业对齐；`EXIT` 允许在尚无扫描地图时执行，普通导航和 `ENTER` 仍遵守地图检查。斜移与直线共用x/y位置PID，单轴进入容差即停止该轴，偏离或过冲后继续纠正；两轴和角度均到达才完成。车体vx/vy分别受 `translation_max_vx_mps` / `translation_max_vy_mps` 限制。`RouteStep.distance_mm` 对斜移表示每轴距离，执行段 `amount` 和计划总距离表示合成斜线长度（总距离取整到mm）。

比赛绑定后订阅 `/mission/heartbeat`，心跳超时停止路线。运行状态中的quiescent/motion_released表示软件执行器已释放输出；实际停车和机构行驶许可还需真实C板反馈。调用方不能将Service接受等同于车辆物理静止。

```text
7 ─ 4 ─ 1
│   │   │
8 ─ 5 ─ 2
│   │   │
9 ─ 6 ─ 3
```

## 构建与启动

```bash
source /opt/ros/humble/setup.bash
cd /root/工创/ros2_ws
colcon build --symlink-install --packages-up-to agv_planning
source /root/工创/ros2_ws/install/setup.bash
ros2 launch agv_planning planner.launch.py \
  config:=/root/工创/ros2_ws/src/agv_planning/config/planner.yaml
```

YAML 是原生 ROS2 参数文件，运行中参数只读；修改后重启生效。

比赛默认的单边行驶距离通过 `config/planner.yaml` 中的 `default_edge_length_mm` 调整，单位为 mm。例如将每条相邻道路从 1200 mm 改为 1000 mm：

```yaml
route_planner_node:
  ros__parameters:
    default_edge_length_mm: 1000
```

只修改现有文件中的这一项，保留其余配置。所有未被 `edge_length_mm` 单独覆盖的路段都会使用新长度，规划距离和运动到达判定同步生效；连续两个同向路段此时合并为 2000 mm。保存后按上面的命令显式加载源 YAML 并重启路径节点，无需重新构建。

## 接口与执行

| 接口 | 类型 | 内容 |
| --- | --- | --- |
| `/route/plan` | `agv_interfaces/srv/PlanRoute` | 规划并执行请求 |
| `/route/path` | `agv_interfaces/msg/RoutePlan` | 节点路线和离散行动链，同一条消息供调试 |
| `/C/odometry` | `agv_interfaces/msg/OdometryFeedback` | 会话固定系 x/y/yaw 与车体系实际速度，由通信节点发布 |
| `/route/control` | `agv_interfaces/msg/RouteControl` | 给mission的速度与执行状态反馈，C板通信不订阅此话题 |
| `/C/controll` | `agv_interfaces/msg/ControlCommand` | mission统一发布的最终底盘速度，由板间通信节点转发C板 |
| `/route/execution_status` | `agv_interfaces/msg/RouteExecutionStatus` | EXECUTING / COMPLETED / ABORTED / REJECTED |
| `/map/blocked_edges` | `agv_interfaces/msg/BlockedEdges` | 整份封锁道路快照 |

里程反馈为固定会话坐标 `x_m/y_m`（m）与连续航向 `yaw_rad/yaw_deg`，另有实际车体系速度、采样时间、有效性、坐标会话和许可代次。按消息时间和接收时间检查新鲜度，并检查 `/C/status` 的 PC/健康许可。

每段平移记录起点和目标航向，将动作方向旋转到固定系。目标位置在复位后首条有效里程建立，后续路段在原目标上累加位移，避免每段按实际停车位置重新起算而累积误差。显式指定起点或朝向的人工调试请求会重新建立位置基准。调试进度仍显示固定目标方向的有符号投影（mm）与横向偏差，控制使用完整x/y目标位置误差。

速度字段为 `vx_mps`（向前正，m/s）、`vy_mps`（向左正，m/s）、`omega_deg_s`（逆时针正，°/s）。自动任务还绑定 `odom_session_id/motion_epoch`，mission 原样转发。
采用明确单位的自定义消息，避免把角度/秒填入规定弧度/秒的 Twist 字段。

`RouteControl.command` 为上述三个速度，`state` 为 `EXECUTING / COMPLETED / ABORTED`；完成和中止反馈中的速度必为零。反馈还包括采样时间、`run_id / request_id / executor_session_id / source`及任务内递增`sequence`，供mission过滤过期、旧任务、错误来源或乱序速度。`/route/execution_status`继续独立保留用于调试及状态核对。Service接受不代表导航完成。

连续同类动作会合并执行：两个前进合成 2400 mm，两个同向转动合成180°，中间不停车。
直线与斜移均使用x/y双轴位置PID，沿x行驶时也纠正y偏差，横移时同理。位置误差和实测速度换算到目标航向对应的固定轴，分别按 `v = Kp × 位置误差 + Ki × 误差积分 - Kd × 实测速度` 计算速度，位置单位m、速度单位m/s。两轴共用增益、分别积分和限幅，当前vx上限0.7 m/s、vy上限0.4 m/s，分别由 `translation_max_vx_mps` / `translation_max_vy_mps` 配置。位置环逐轴限制输出，换算到当前车体系后再次限制，保证 `|vx| <= translation_max_vx_mps`、`|vy| <= translation_max_vy_mps`。某轴输出饱和且积分会加剧饱和时暂停该轴积分，另一轴仍独立积分；每轴进入位置容差后输出零并清积分，过冲可反向修正。
角度环按 `omega = Kp × (目标角度 - 当前角度) + Ki × 误差积分 - Kd × 实测角速度` 输出 °/s，绝对值不超过 `angle_max_speed_deg_s`（当前120°/s）。D项使用里程反馈的实测角速度，避免切换目标角度时微分突跳；积分使用实际控制间隔，输出饱和且积分会加剧饱和时暂停累积。进入角度容差后输出零并清积分。
目标航向在复位后的首条有效里程建立，跨动作及正常路线请求保留；转向在原目标上累加有符号转角，过冲后允许反向修正。显式指定起点或朝向的人工调试请求会以当前反馈重新建立航向基准。角度不回绕，跨360°及合并180°转向仍保持规划方向。
前进、后退、横移和斜移同时运行位置环与角度环，将位置环输出换算到当前车体系，边纠偏边锁头。两条位置轴的绝对误差均不超过 `translation_arrival_tolerance_mm`（当前±3 mm），且角度进入 `turn_arrival_tolerance_deg` 才完成本段；位置到达后若因转动再次偏离，位置环会继续修正。不同动作之间和最终到达时发布零速度；后退优先于掉头再前进，只有终点要求作业朝向时才转头。

执行期间拒绝新请求，保留当前路线。里程/状态消息超时、无效、坐标/运动许可变化，或任一未到达的位置轴、角度误差未缩小时停车并中止，不自动续跑。逐轴监测可检测侧向卡住或斜移单轴不响应，其他轴的推进不能掩盖故障。超时由 `motion_progress_timeout_s` 决定。
坐标/运动许可变化后先人工确认当前位置与朝向，通过执行器上下文复位再发新任务，无需重启规划器。路线与视觉任务由调度状态机分配到不同阶段；本节点待命时不持续发布零速度。

## 参数与朝向

配置文件 `config/planner.yaml`：

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `start_area` | `left` | 面向二维码板时，左侧启动区1为left，右侧启动区3为right；开局到2均为前进 |
| `work_heading.node_2/4/6/8` | YAML：`DOWN/LEFT/RIGHT/DOWN`；未加载配置时 `UNSET` | 四个工作区作业时的车头朝向 |
| `default_edge_length_mm` | 1200 | 相邻节点距离，整数mm |
| `edge_length_mm.<小编号>_<大编号>` | 默认边长 | 无向边实测距离覆盖 |
| `translation_pid_kp` / `translation_pid_ki` / `translation_pid_kd` | 2.0 / 0.0 / 0.1 | x/y共用位置环增益，分别为1/s、1/s²、无量纲；初值需实车整定，Ki=0关闭积分 |
| `translation_max_vx_mps` | 0.7 | 车体前后速度绝对值上限，m/s，包含纠偏速度与斜移 |
| `translation_max_vy_mps` | 0.4 | 车体左右速度绝对值上限，m/s，包含纠偏速度与斜移 |
| `translation_arrival_tolerance_mm` | 3.0 | 两条位置轴各自允许的绝对误差，±mm；两轴均到达才通过位置判定 |
| `angle_pid_kp` / `angle_pid_ki` / `angle_pid_kd` | 2.0 / 0.0 / 0.1 | 角度环增益，分别为1/s、1/s²、无量纲；初值需实车整定，Ki=0关闭积分 |
| `angle_max_speed_deg_s` | 120.0 | 全部运动共用的PID角速度绝对值上限，°/s |
| `turn_arrival_tolerance_deg` | 0.2 | 全部运动共用的角度允许误差，±°；平移还需距离到达 |
| `odometry_timeout_s` | 0.5 | 接收里程反馈的最大间隔 |
| `motion_progress_timeout_s` | 2.0 | 任一未到达位置轴或角度误差无缩小时的停车时限 |
| `control_rate_hz` | 50.0 | 控制发布频率 |
| `require_map_snapshot` | true | 未收到合法地图时拒绝请求；false用于明确全通调试 |

话题名及 Service 名也可在 YAML 修改。朝向为地图的 UP / RIGHT / DOWN / LEFT。

旧的 `translation_fast_mps / translation_slow_mps / translation_deceleration_mm` 和 `turn_fast_deg_s / turn_slow_deg_s / turn_deceleration_deg` 已分别由位置PID和角度PID参数替代。修改代码后需重新构建 `agv_planning` 并重启；以后只改 YAML 时重启即可。`control_rate_hz` 默认50 Hz，是ROS定时控制频率，不是硬实时调度保证。
`start=0` 从规划器当前记录的节点出发，空 `current_heading` 使用反馈 yaw 推导朝向。
`goal=0` 回到配置启动区。显式 `start=1..9` 需要提供 `current_heading`。
`align_at_goal=true` 只支持2、4、6、8四个工作区，终点朝向可从 YAML 或请求 `goal_heading` 提供；途中经过工作区不转头。

`work_heading` 和请求中的 `goal_heading` 始终表示车头（+vx）的地图方向。当前约定二维码区用车左侧（+vy）面向二维码板，其余原料、粗加工、暂存区用车右侧（-vy）面向工作区，因此 YAML 中车头方向依次为2向下、4向左、6向右、8向下。到站后按实际朝向执行所需自转，已满足时不转；区间平移保持车头朝向。

## 调试例子

每个新终端先 source ROS2 与工作空间。先观察输出：

```bash
ros2 topic echo /route/path
ros2 topic echo /route/control
ros2 topic echo /route/execution_status
```

发布整份地图，空列表表示全通：

```bash
ros2 topic pub --once /map/blocked_edges agv_interfaces/msg/BlockedEdges \
  '{map_revision: 1, edges: []}'
```

板间通信节点发布真实二维里程和当前 C 板状态；不要用固定数值冒充真实反馈。纯软件验证使用仓库 PTY 测试，假板应同时上报与实际运动对应的 x/y、yaw 和车体系速度。

首次从启动区前进到二维码区，不要求终点额外转向：

```bash
ros2 service call /route/plan agv_interfaces/srv/PlanRoute \
  "{request_id: 'stage1-001', stage: 1, start: 0, goal: 2, current_heading: '', align_at_goal: false}"
```

Service 返回并在 `/route/path` 发布同一份计划，`status: ok` 表示已接受执行；真实完成必须等对应 `request_id` 的 `COMPLETED`。
独立启动路径节点只输出内部控制反馈，不直接控制底盘；比赛中需配合mission及真实板间通信。原配置中的 `control_topic: /C/controll` 必须改为 `/route/control`，内部反馈消息类型也已改变，修改后重新构建并重启相关节点。
失败状态包括 `no_path`、`invalid_request`、`map_not_ready`、`odometry_not_ready`、`odometry_invalid`、`execution_in_progress` 和 `duplicate_request_id`。节点运行期间会记录所有已接受的 `request_id`，重复 ID 不会再次触发运动；新任务必须使用新 ID，重启节点会清空记录。
离散行动码：1前进、2左转90°、3右转90°、4左移、5右移、6后退、7前左斜移、8前右斜移、9后左斜移、10后右斜移。新增请求字段和行动码后需重新构建 `agv_interfaces` 及相关节点并重启。

规划按 `(总路程, 移动方向变化次数)` 最小化，再按节点序列稳定选取同分路线。
默认等长地图中 `7896` 比 `7856` 少一次方向变化；全通时同分的 `7456` 按节点序列优先，封锁7↔4时选择 `7896`。
地图必须为整份快照，revision为正且单调递增；同版本同内容允许重发，旧版或同版不同内容被拒绝。


## 二维执行验证

ROS2 和 workspace 已 source 后，在独立 domain 运行：

```bash
python3 -m unittest discover -s /root/工创/ros2_ws/src/agv_planning/test -p test_xy_execution.py -v
```

普通路线的 `RoutePlan.total_distance_mm` 仍是拓扑计划总长度；删除的是实测反馈累计路程。`RouteExecutionProgress` 改为段起点、当前点、固定方向、投影进度和横向偏差。平移进度单位 mm、转向进度单位 deg。
