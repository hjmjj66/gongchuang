# 3×3 路径规划与里程计执行节点

面向 ROS2 Humble，包 `agv_planning`，可执行文件 `route_planner_node`。
节点等待地图、二维里程和规划请求，成功规划后自动执行。路线与离散行动链仅作调试查看，实际运动通过速度话题完成。

浏览器实车调试入口见 [tools/route_debug](../../../tools/route_debug/README.md)。工具启动时单独开启 `debug_allow_any_pose`（允许1～9任意初始节点和四个方向）与 `debug_publish_progress`（10 Hz `/route/progress`，`RouteExecutionProgress`，工具内映射到私有话题）。两项正式默认均为false，保留比赛启停区复位限制；进度只供显示，不参与控制。

路径节点作为mission调用的导航工具：mission只指定目标节点及是否作业对齐，起点和当前朝向仍由路径节点内部确定。路径节点向 `/route/control` 返回速度和执行状态，由mission统一转发速度到 `/C/controll`；C板通信只订阅mission的最终输出。

比赛接入新增 `/route/get_runtime`、`/route/stop`、`/route/reset_context`，分别为 `GetExecutorRuntime`、`StopExecutor`、`ResetExecutor` 服务，类型位于 `agv_interfaces`。运行查询包括当前任务、最近终态、进程身份、节点/朝向、里程健康、地图版本及封锁边。

`PlanRoute`、`RoutePlan`、`RouteExecutionStatus`、`BlockedEdges` 新增 `run_id`。未绑定比赛时可省略以保持独立调试用法；比赛初始化后仅接受匹配身份，比赛路线使用 `start=0` 和反馈朝向。地图也必须携带本轮身份。旧程序须重新构建以匹配更新后的消息。

停止服务按活动身份停止速度控制循环并归零，中途停止后位置未知。复位使用 `expected_run_id / run_id / reset_id` 实现比较并交换及幂等性，仅在空闲时接受，清地图与旧里程检查基线，并等待下一条新里程建立yaw基准。复位代表人工已恢复初始现场，不自动返航。

比赛的 `start_area / finish_area` 统一放在 `agv_mission/config/mission.yaml`。mission初始化/复位时同步起点，最终停车区作为最后一次导航的明确目标发送；planner YAML不再重复配置起点。单独调试规划器且未绑定mission时，仍默认从节点1启动，可用节点参数 `start_area:=right` 改为节点3；这项调试默认值会在比赛初始化时被mission覆盖。

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

每段平移记录起点和开始航向，将动作方向旋转到固定系，计算位移在固定目标方向的有符号投影（mm）。横向位移、反向运动和原地转动不能累计为前进；横向偏差只显示，本轮不自动纠偏或锁头。旋转仍比较连续 yaw。

速度字段为 `vx_mps`（向前正，m/s）、`vy_mps`（向左正，m/s）、`omega_deg_s`（逆时针正，°/s）。自动任务还绑定 `odom_session_id/motion_epoch`，mission 原样转发。
采用明确单位的自定义消息，避免把角度/秒填入规定弧度/秒的 Twist 字段。

`RouteControl.command` 为上述三个速度，`state` 为 `EXECUTING / COMPLETED / ABORTED`；完成和中止反馈中的速度必为零。反馈还包括采样时间、`run_id / request_id / executor_session_id / source`及任务内递增`sequence`，供mission过滤过期、旧任务、错误来源或乱序速度。`/route/execution_status`继续独立保留用于调试及状态核对。Service接受不代表导航完成。

连续同类动作会合并执行：两个前进合成 2400 mm，两个同向转动合成180°，中间不停车。
直线默认以 1 m/s 匀速行驶，最后100 mm使用匀减速关系 `v²=v1²+(v0²-v1²)×(剩余距离-到达容差)/(减速距离-到达容差)`，减至配置慢速，进入到达容差立即发零。
转动速度同样以实际加载 YAML 为准（当前配置 60°/s、30°/s），最后5°减速；90°转向从85°开始减速，180°转向从175°开始。
不同动作之间和最终到达时发布零速度。平移仅请求 omega=0（本轮不闭环锁头），后退优先于掉头再前进，只有终点要求作业朝向时才转头。

执行期间拒绝新请求，保留当前路线。里程/状态消息超时、无效、坐标/运动许可变化，或目标方向投影/yaw在设定时限内没有推进时停车并中止，不自动续跑。
坐标/运动许可变化后先人工确认当前位置与朝向，通过执行器上下文复位再发新任务，无需重启规划器。路线与视觉任务由调度状态机分配到不同阶段；本节点待命时不持续发布零速度。

## 参数与朝向

配置文件 `config/planner.yaml`：

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `start_area` | `left` | 面向二维码板时，左侧启动区1为left，右侧启动区3为right；开局到2均为前进 |
| `work_heading.node_2/4/6/8` | `UNSET` | 四个工作区作业朝向 |
| `default_edge_length_mm` | 1200 | 相邻节点距离，整数mm |
| `edge_length_mm.<小编号>_<大编号>` | 默认边长 | 无向边实测距离覆盖 |
| `translation_fast_mps` / `translation_slow_mps` | 1.0 / 0.5 | 平移快慢速度 |
| `translation_deceleration_mm` | 100.0 | 平移减速区长度 |
| `translation_arrival_tolerance_mm` | 5.0 | 平移到达容差 |
| `turn_fast_deg_s` / `turn_slow_deg_s` | 60.0 / 30.0 | 转向快慢角速度 |
| `turn_deceleration_deg` | 5.0 | 转向减速区角度 |
| `turn_arrival_tolerance_deg` | 0.25 | 转向到达容差 |
| `odometry_timeout_s` | 0.5 | 接收里程反馈的最大间隔 |
| `motion_progress_timeout_s` | 2.0 | 正在运动但方向投影或yaw没有推进时的停车时限 |
| `control_rate_hz` | 50.0 | 控制发布频率 |
| `require_map_snapshot` | true | 未收到合法地图时拒绝请求；false用于明确全通调试 |

话题名及 Service 名也可在 YAML 修改。朝向为地图的 UP / RIGHT / DOWN / LEFT。
`start=0` 从规划器当前记录的节点出发，空 `current_heading` 使用反馈 yaw 推导朝向。
`goal=0` 回到配置启动区。显式 `start=1..9` 需要提供 `current_heading`。
`align_at_goal=true` 只支持2、4、6、8四个工作区，终点朝向可从 YAML 或请求 `goal_heading` 提供；途中经过工作区不转头。

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
离散行动码：1前进、2左转90°、3右转90°、4左移、5右移、6后退。

规划按 `(总路程, 移动方向变化次数)` 最小化，再按节点序列稳定选取同分路线。
默认等长地图中 `7896` 比 `7856` 少一次方向变化；全通时同分的 `7456` 按节点序列优先，封锁7↔4时选择 `7896`。
地图必须为整份快照，revision为正且单调递增；同版本同内容允许重发，旧版或同版不同内容被拒绝。


## 二维执行验证

ROS2 和 workspace 已 source 后，在独立 domain 运行：

```bash
python3 -m unittest discover -s /root/工创/ros2_ws/src/agv_planning/test -p test_xy_execution.py -v
```

`RoutePlan.total_distance_mm` 仍是拓扑计划总长度；删除的是实测反馈累计路程。`RouteExecutionProgress` 改为段起点、当前点、固定方向、投影进度和横向偏差。平移进度单位 mm、转向进度单位 deg。
