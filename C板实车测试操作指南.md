# C 板二维里程与路线执行实车测试操作指南

更新日期：2026-10-26。本文用于验证 C 板通信、二维里程、手动速度、自由行走和模拟比赛。改造说明见 [C板通信与二维里程反馈改造实施记录](docs/C板通信与二维里程反馈改造实施记录.md)，本次方向修正见 [车体坐标统一实施记录](docs/车体坐标统一实施记录.md)。

本文中的速度指令会驱动实车。首次测试每一种新运动时先架空车轮核对方向；手动示例使用 `0.05 m/s`，路线运行使用 `planner.yaml` 中设置的实际速度，现场保留遥控器或急停。异常时优先切遥控或急停，再停止软件。

## 1. 测试顺序与统一约定

按顺序执行，前一步正常后再进行下一步：

1. 构建并确认通信健康。
2. 手动测试前后、左右和旋转。
3. 用诊断工具分别执行定时运动、固定距离和固定角度运动。
4. 浏览器“自由行走”模式：人工选起点和终点后启动。
5. 浏览器“模拟比赛”模式：执行八段路线和七次三秒等待。
6. 可选：静止采集 yaw 漂移，提供给电控分析。

| 字段 | 正方向 | 单位 |
| --- | --- | --- |
| `vx_mps` | 当前车头前方 | m/s |
| `vy_mps` | 当前车身左侧 | m/s |
| `omega_deg_s` | 逆时针 | °/s |
| `/C/odometry.x_m` | 进入 PC 时车头前方建立的固定 +x | m |
| `/C/odometry.y_m` | 进入 PC 时车身左侧建立的固定 +y | m |

位置坐标轴在进入 PC、通信节点重启、串口重连或检测到板端重启后重新建立；车转向后位置坐标轴不随车旋转。`vx_mps/vy_mps` 始终是当前车体系的实测速度。

## 2. 构建工作空间

在运行 ROS2 的设备上执行：

```bash
source /opt/ros/humble/setup.bash
cd /root/工创/ros2_ws
colcon build --symlink-install
source /root/工创/ros2_ws/install/setup.bash
```

每个新终端都重新执行：

```bash
source /opt/ros/humble/setup.bash
source /root/工创/ros2_ws/install/setup.bash
```

不要设置自动化测试使用的 ROS 域 `92/93/94`。可确认接口和诊断工具：

```bash
ros2 interface show agv_interfaces/msg/OdometryFeedback
ros2 run agv_cboard cboard_diagnostics --help
```

接口应包含：`x_m`、`y_m`、`yaw_rad`、`yaw_deg`、`vx_mps`、`vy_mps`、`wz_rad_s`。

## 3. 现场准备与进程检查

1. 清空车辆周围区域。
2. 确认串口是 `/dev/ttyACM0`，车辆供电正常。
3. 确认遥控器可接管，急停可用。
4. 结束旧的 mission、桥接、规划器、调试工具和所有 `ros2 topic pub`。
5. 同一时间只能有一个最终速度发布者向 `/C/controll` 发非零命令。

```bash
ros2 node list
ros2 topic info -v /C/controll
```

手动测试时应只有桥接节点订阅 `/C/controll`，没有其他非预期发布者。

## 4. 启动通信节点并确认健康

### 4.1 终端 A：板件通信节点

```bash
source /opt/ros/humble/setup.bash
source /root/工创/ros2_ws/install/setup.bash
ros2 launch agv_cboard cboard.launch.py serial_port:=/dev/ttyACM0
```

等待日志出现串口连接和：

```text
C-board feedback healthy; PC control available
```

### 4.2 终端 B：状态

```bash
source /opt/ros/humble/setup.bash
source /root/工创/ros2_ws/install/setup.bash
ros2 topic echo /C/status
```

将遥控器拨到 PC，等待以下状态：

```yaml
connected: true
status_valid: true
odometry_valid: true
pc_control: true
motion_allowed: true
origin_initialized: true
control_source: 2
fault_reason: ''
```

`motion_allowed` 表示可以接收运动指令，不代表车辆物理停稳。切 PC 后会建立新的 `odom_session_id`，第一条健康 ODOM 后局部位置归零。

### 4.3 终端 C：里程

```bash
source /opt/ros/humble/setup.bash
source /root/工创/ros2_ws/install/setup.bash
ros2 topic echo /C/odometry
```

另开终端查看原始板端数据：

```bash
ros2 topic echo /C/chassis_odometry
```

遥控期间仍应有新观测；但 `motion_allowed` 应为 `false`。如果不健康，先处理 `fault_reason`，不要继续运动测试。

| 现象 | 检查 |
| --- | --- |
| `connected: false` | 串口、USB、权限、供电 |
| `control_source != 2` | 遥控器切 PC |
| `origin_initialized: false` | 等待健康 ODOM，检查原始话题 |
| `odometry_valid: false` | ODOM 状态位、轮/IMU/编码器错误 |
| `control_source: 2` 但故障仍不清除 | 在 Foxglove 查看 `/C/status` 与原始里程，停止运动并排查 |

## 5. 按 vx / vy / w 运行固定时间（直接复制）

完成第 2 节构建，保持终端 A 的通信节点运行。关闭 mission、浏览器工具、旧诊断命令和 `ros2 topic pub`，一次只运行下面**一个**代码块；不要把所有运动代码块连续粘贴执行。每块都包含环境加载和可修改的运动参数，可以整块复制到新的命令终端。所有示例默认直接执行，不生成采样记录文件；位置、速度、角度和状态由你在另一个终端连接 Foxglove 实时监测。

这里的 `w` 对应命令行 `--omega`，单位是 **°/s**，不是 rad/s。代码块上方的变量可以直接修改：`DURATION_S` 是实际持续发速度的秒数，`VX_MPS/VY_MPS/W_DEG_S` 是速度值。未使用的轴显式写零，不会附带其他方向的运动。

工具会先等待 PC、健康反馈和订阅匹配，再从第一次发布速度开始计时，约 50 Hz 发令，结束后主动发零。启动等待时间不算入 `DURATION_S`，因此不要再在外面套 `timeout 3s`。

### 5.1 向前：vx = +0.05 m/s，持续 3 秒

```bash
source /opt/ros/humble/setup.bash
source /root/工创/ros2_ws/install/setup.bash

DURATION_S=3
VX_MPS=0.05

ros2 run agv_cboard cboard_diagnostics pulse \
  --duration "$DURATION_S" \
  --vx "$VX_MPS" --vy 0 --omega 0 \
  --wait 15
```

向后时只把 `VX_MPS=0.05` 改成 `VX_MPS=-0.05`。理想位移大小是 `0.05 × 3 = 0.15 m`；实际位移受加减速和打滑影响，定时命令不按距离停车。

### 5.2 向左：vy = +0.05 m/s，持续 3 秒

```bash
source /opt/ros/humble/setup.bash
source /root/工创/ros2_ws/install/setup.bash

DURATION_S=3
VY_MPS=0.05

ros2 run agv_cboard cboard_diagnostics pulse \
  --duration "$DURATION_S" \
  --vx 0 --vy "$VY_MPS" --omega 0 \
  --wait 15
```

向右时只把 `VY_MPS=0.05` 改成 `VY_MPS=-0.05`。

### 5.3 原地逆时针：w = +10°/s，持续 3 秒

```bash
source /opt/ros/humble/setup.bash
source /root/工创/ros2_ws/install/setup.bash

DURATION_S=3
W_DEG_S=10

ros2 run agv_cboard cboard_diagnostics pulse \
  --duration "$DURATION_S" \
  --vx 0 --vy 0 --omega "$W_DEG_S" \
  --wait 15
```

顺时针时只把 `W_DEG_S=10` 改成 `W_DEG_S=-10`。理想转角是 `10 × 3 = 30°`；需要按实际转角停车时用第 6.3 节。

### 5.4 中止、停车与异常判断

在运动命令终端按 `Ctrl+C` 可提前结束，工具会尝试主动发零。需要立即接管时使用遥控器／急停。结束运动发布进程后，可整块复制以下命令补发零速：

```bash
source /opt/ros/humble/setup.bash
source /root/工创/ros2_ws/install/setup.bash

ros2 topic pub --once /C/controll agv_interfaces/msg/ControlCommand \
  '{vx_mps: 0.0, vy_mps: 0.0, omega_deg_s: 0.0}'
```

另开终端查看最近实际写入串口的速度：

```bash
source /opt/ros/humble/setup.bash
source /root/工创/ros2_ws/install/setup.bash
ros2 topic echo /C/status
```

- 前进应只有 `sent_vx_mps` 非零，左移应只有 `sent_vy_mps` 非零，旋转应只有 `sent_wz_rad_s` 非零。这里的 `sent_wz_rad_s` 已是 rad/s，输入 `10°/s` 对应约 `0.174533 rad/s`。
- 正常运动时 `sent_mode=1`，命令结束后恢复全零、`sent_mode=0`。这表示串口写入内容，不保证物理停稳。
- `Other /C/controll publisher` 或 `Competing command publisher`：结束其他速度发布者再重试。
- `Timed out waiting`：检查是否切 PC、反馈是否健康、通信节点是否运行。
- `input timed out`：输入中断；检查诊断进程是否运行以及 CPU／DDS 是否阻塞。
- 实际速度有值但 x/y 不变：在 Foxglove 对比原始与局部里程，排查板端位置积分，暂不使用固定距离命令。

## 6. 按反馈走固定 x / y 距离或固定角度（直接复制）

本节使用新增的 `cboard_diagnostics move`。先执行第 2 节构建并重新 source，重启旧进程；可用 `ros2 run agv_cboard cboard_diagnostics --help` 确认存在 `move`、`--dx`、`--dy`、`--angle`。

与第 5 节的区别：工具记录本次起点，读取 `/C/odometry`，达到目标距离或角度的停止阈值后发零；**不是用距离除以速度换算时间**。每次相对于本次开始位置计量，不要求把里程归零。一次只选择 `--dx`、`--dy`、`--angle` 中的一项。

| 参数 | 含义 |
| --- | --- |
| `--dx 0.20` | 沿本次开始时的车头前向移动 0.20 m；负数向后 |
| `--dy 0.20` | 沿本次开始时的车身左向移动 0.20 m；负数向右 |
| `--angle 90` | 相对本次开始航向逆时针转 90°；负数顺时针；连续角支持大于 360° |
| `--speed 0.05` | 平移速度上限 0.05 m/s，填正数；方向由 dx/dy 的符号决定 |
| `--omega 10` | 转动角速度上限 10°/s，填正数；方向由 angle 的符号决定 |
| `--tolerance-m 0.005` | 距离剩余不超过 5 mm 时开始发零，不是实际停车精度保证 |
| `--tolerance-deg 1` | 角度剩余不超过 1° 时开始发零，不是实际停车精度保证 |
| `--timeout 30` | 从开始执行起最多允许运动 30 秒；未到目标则报错停车 |
| `--stall-timeout 3` | 连续 3 秒没有足够的目标方向进展则报错停车 |
| `--wait 15` | 开始前等待通信健康和订阅匹配，最多 15 秒，不计入运动时间 |

**坐标含义要区分：**本节 dx/dy 是“本次起步时前／左方向”的相对位移投影；`/C/odometry.x_m/y_m` 是进入 PC 时建立的固定坐标。如果车辆之后已经转向，执行 `--dx/--dy` 时固定 x 和 y 可能同时变化，这是正常坐标投影，并不直接表示车身斜走。例如本次车头朝向相对 PC 原点逆时针 90°，前进会主要增加固定 y。若要让 `--dx/--dy` 恰好对应屏幕固定 x/y 轴，摆正车后切 RC 再切 PC、等健康，再开始测试。

### 6.1 沿本次开始时的前向移动 0.20 m

整块复制；只修改 `DX_M` 可以调整方向和距离，`SPEED_MPS` 始终保持正数。

```bash
source /opt/ros/humble/setup.bash
source /root/工创/ros2_ws/install/setup.bash

DX_M=0.20
SPEED_MPS=0.05
TOLERANCE_M=0.005
TIMEOUT_S=30

ros2 run agv_cboard cboard_diagnostics move \
  --dx "$DX_M" --speed "$SPEED_MPS" \
  --tolerance-m "$TOLERANCE_M" \
  --timeout "$TIMEOUT_S" --stall-timeout 3 --wait 15
```

向后 0.20 m：把 `DX_M=0.20` 改成 `DX_M=-0.20`，其余不变。

### 6.2 沿本次开始时的左向移动 0.20 m

```bash
source /opt/ros/humble/setup.bash
source /root/工创/ros2_ws/install/setup.bash

DY_M=0.20
SPEED_MPS=0.05
TOLERANCE_M=0.0015
TIMEOUT_S=30

ros2 run agv_cboard cboard_diagnostics move \
  --dy "$DY_M" --speed "$SPEED_MPS" \
  --tolerance-m "$TOLERANCE_M" \
  --timeout "$TIMEOUT_S" --stall-timeout 3 --wait 15
```

向右 0.20 m：把 `DY_M=0.20` 改成 `DY_M=-0.20`。例如左移 0.50 m 就改成 `DY_M=0.50`；较长距离还需相应增加 `TIMEOUT_S`，因为接近终点会减速。

### 6.3 原地逆时针转 90°

```bash
source /opt/ros/humble/setup.bash
source /root/工创/ros2_ws/install/setup.bash

ANGLE_DEG=-90
W_MAX_DEG_S=30
TOLERANCE_DEG=0.12
TIMEOUT_S=30

ros2 run agv_cboard cboard_diagnostics move \
  --angle "$ANGLE_DEG" --omega "$W_MAX_DEG_S" \
  --tolerance-deg "$TOLERANCE_DEG" \
  --timeout "$TIMEOUT_S" --stall-timeout 3 --wait 15
```

顺时针 90°：改成 `ANGLE_DEG=-90`；逆时针 180°：改成 `ANGLE_DEG=180`。这里 `--omega` 是正数速度上限，**不要通过改成负速度来选择方向**。

### 6.4 到点后的输出、限制与异常

正常输出包含两部分：

```text
Target threshold reached: displacement=...       # 达到发零的阈值
After 0.5s zero command: measured=..., target=..., error=...
```

第二行报告持续发零约 0.5 秒后测得的最终相对位移／转角、误差，以及固定系 dx/dy 和相对 dyaw。观察车辆是否实际停稳，并在 Foxglove 查看最终位置和角度；不要只把第一行当作实际停车位置。

- 最后 5 cm 平移或 15° 旋转逐渐减速；默认低速下限分别为 0.01 m/s、2°/s（不会超过你设置的上限）。如果车辆低速死区导致无进展，工具会报错停车；先记录给电控，不要无限延长超时。
- 距离按目标方向的**有符号净位移**计算。横向偏移不计作前进，反向移动不会被当成完成，也不会累加来回路程。
- 这是单轴里程阈值停车，未增加横向纠偏或平移航向保持，也不会在过冲后自动倒车反复找点。误差受里程精度、打滑、低速响应和惯性影响，需要用尺子／角度参考核对真实运动。
- 切 RC、串口断开、里程／状态过期、坐标会话变化、竞争发布者出现、无进展或总时长超限都会中止；不会自动续跑。处理后重新发命令，会以新的开始位置计量。
- `Motion timeout; target was NOT reached` 和 `No progress` 都表示失败，不是到点成功。需要重试前确认车已停稳。
- 提前中止同样按 `Ctrl+C`，必要时按第 5.4 节遥控接管并补发零速。

## 7. 可选：静止 yaw 漂移采样

车辆静止、没有速度发布者时：

```bash
ros2 run agv_cboard cboard_diagnostics monitor \
  --duration 60
```

该命令只观察，运行 60 秒后在终端输出 yaw 斜率、平均 `wz_rad_s` 和积分。期间可在 Foxglove 实时观察原始／局部里程和状态。若物理静止时原始 yaw 仍漂移，把终端统计值交给电控检查陀螺零偏、标定、积分周期和融合。

## 8. 浏览器工具：自由行走模式

浏览器工具会自己启动桥接节点、规划器和控制器。因此先在第 4 节终端按 `Ctrl+C` 结束单独桥接，且不要启动 mission。

### 8.1 直接修改 planner.yaml

浏览器工具默认直接读取 `/root/工创/ros2_ws/src/agv_planning/config/planner.yaml`。速度、减速区、到达容差和道路长度都在这个文件中修改：

```bash
nano /root/工创/ros2_ws/src/agv_planning/config/planner.yaml
```

保留 `route_planner_node` → `ros__parameters` 层级，在其中按实际需要调整参数。当前路线速度配置为：

```yaml
translation_fast_mps: 0.8
translation_slow_mps: 0.5
turn_fast_deg_s: 60.0
turn_slow_deg_s: 30.0
```

单边默认 1200 mm，减速区和到达容差也由此文件决定；按实际运行速度留足空间。

参数在启动时读取。保存后按第 8.2 节启动；如果工具已运行，先在终端按 `Ctrl+C` 结束，再重新启动。仅修改这个 YAML 无需重新构建。后续自由行走和模拟比赛都会使用该文件中的最新配置。

打滑处理是通信桥接参数，直接编辑 `/root/工创/ros2_ws/src/agv_cboard/config/cboard.yaml` 中的 `stop_on_slip`；当前为 `false`，C 板单独上报打滑只记录告警，不再因此中止路线。急停、轮掉线、IMU/编码器故障、反馈超时和坐标变化仍会停车。页面及 Foxglove 的 `odom_status` 保留打滑标志，继续运行时实际位置仍可能因打滑产生误差。桥接没有低速或 `2 m/s` 门限；若需调整 C 板的打滑判定门限，需要修改板端固件中的角速度差判据。修改此 YAML 后也须重启工具。

### 8.2 启动工具并测试节点 1→2

```bash
cd /root/工创
./tools/route_debug/start.sh \
  serial_port:=/dev/ttyACM0 \
  port:=8080
```

打开 `http://127.0.0.1:8080`。其他电脑使用 `http://小车IP:8080`。页面四项健康状态都应为正常：C 板连接、PC 控制、有效里程、运动许可。

在页面“实际加载的运动配置”确认来源为上述 `planner.yaml`，并核对平移、转动快慢速与文件中的设置一致。

首次自由行走：

1. 车辆人工放在节点 1，车头朝地图下方。
2. 页面选择“自由行走”。
3. 起点选“节点 1”，终点选“节点 2”，朝向选“地图下方”。
4. 第一次不勾选“到终点后按固定工作方向对齐”。
5. 点击“启动”。

预期：`初始化中 → 导航中 → 已完成`。这一段应向当前车头前方行驶，下发 `vx>0、vy=0、omega=0`；若本次 PC 会话刚在当前姿态建立，位置主要增加固定 x。页面检查固定系 x/y、实际速度、指令速度、本段投影进度、横向偏差、会话/代次；完成后速度归零，轨迹按实测 x/y 绘制而不是吸附到终点。

需要提前停下时点击“停止”，必要时遥控接管。停止、切 RC、串口重连或会话变化后旧任务不会续跑；确认实际位置和朝向后重新选择并启动。

## 9. 浏览器工具：模拟比赛模式

该模式会驱动真实底盘，但跳过扫码、雷达、抓放和机械臂动作。自由行走完成或安全停止后再开始：

1. 在工具终端按 `Ctrl+C`，结束上一轮工具。
2. 人工将车辆放回启停区 1（节点 1）或启停区 2（节点 3）。
3. 节点 1 车头朝地图下方；节点 3 车头朝地图上方。
4. 切换到 PC，重新建立里程原点。
5. 用第 8.2 节命令重新启动工具，等待四项健康状态正常。
6. 选择“模拟比赛”，选择起点和最终停车区，点击“启动”。

路线为：

```text
起始启停区 → 2 → 4 → 6 → 8 → 4 → 6 → 8 → 最终启停区
```

共 8 段导航、7 次等待，每次 3 秒。页面预期依次出现 `导航 1/8 → 等待 → … → 导航 8/8 → 已完成`。观察每段停车、工作节点朝向（2右、4上、6下、8左）、实际 x/y、yaw、横向偏差和速度。比赛中不要再启动其他控制节点。

异常时先遥控/急停，再点击“停止”，等待停止确认；不要直接再次点击“启动”。重新开始前重新摆车、选择位置/朝向并建立新任务。

## 10. Foxglove 实时监测对照

你可以在独立终端运行已有的 Foxglove 连接，实时订阅以下话题。第 4 节的 `ros2 topic echo` 只是终端观察方式，使用 Foxglove 时可以省略这些 echo 命令。

| 话题 | 重点观察 |
| --- | --- |
| `/C/controll` | 本次下发的 `vx_mps / vy_mps / omega_deg_s` |
| `/C/status` | `motion_allowed`、`fault_reason`、`command_reason`；`sent_vx_mps / sent_vy_mps / sent_wz_rad_s / sent_mode` |
| `/C/odometry` | 局部固定系 `x_m / y_m / yaw_deg` 和实际 `vx_mps / vy_mps / wz_rad_s` |
| `/C/chassis_odometry` | 板端原始位置、航向、实际速度，用于与局部坐标对比 |

定时测试关注持续时间、速度轴和结束后的归零；定距／定角测试关注相对起点的净位移或转角，以及停车后的最终值。ROS 指令 `omega_deg_s` 为 °/s，实际反馈和 `sent_wz_rad_s` 为 rad/s，绘图比较前统一单位。

## 11. 停止与退出

| 场景 | 操作 |
| --- | --- |
| 手动命令异常 | 遥控/急停，终止发布进程，再发零速。 |
| pulse / move 异常 | `Ctrl+C`；工具会尝试发零速，但现场仍以遥控/急停为先。 |
| 浏览器路线异常 | 点击“停止”，必要时遥控接管；不要只关闭页面。 |
| 结束浏览器工具 | 在 `start.sh` 终端按 `Ctrl+C`，它会统一结束子进程。 |
| 开始新路线 | 停止确认后重新确认车辆位置/朝向；旧任务不能续跑。 |
