# 粗加工龙门架执行端使用与调试

圆环处理按阶段选择精定位目标：入站/离站纠偏与底盘2号环定位保留三个圆环的几何检测；逐件 `ALIGN_RING` 只对本件圆环找圆，其余YOLO框使用框中心（`center_source=bbox`、`geometry_reason=NOT_TARGET`）。目标丢失不对其他环补做找圆；原有目标定位失败回退规则保留。正式宿主与8086调试均按当前任务目标切换，并拒绝把其他目标精定位期间的框中心用于当前对准计数。

正式入口为 `coarse_node`，任务接口 `/vision/coarse/run_batch`，比赛阶段6、12各调用一次。完整设计见[新版指导计划](../../../docs/主相机视觉方案/14_粗加工区龙门架改造指导计划.md)。该节点同时承载原料、暂存任务，共享 `/vision` 运行/停止/复位接口和相机，不要重复启动宿主。

## 执行流程

```text
高位WORK → 圆心直线斜率纠偏（最多3秒）
→ 停稳记录P0 → 高位用底盘对准2号环 → 底盘停车
→ 按输入顺序逐件：保持工作位 → 按机构反馈前往当前环标定点并停稳 → 机构旋转/伸缩视觉对准
  → 连续4张新帧中心和两轴输出达标，或PID累计达到共用时限 → 停止
  → 记录实际fb_pulses/机构yaw_rad → 固定放置并自动回高位
→ 按同顺序逐件：实际反馈闭环返回记录的长度/角度
  → 停止复核 → 固定夹取、入仓并自动回高位
→ WORK恢复高位 → 斜率纠偏（最多3秒）→ INIT收拢
→ 返回P0并停车，或按请求原地停稳交接导航 → 完成
```

初始环2校准和逐件视觉对准均在WORK工作位置进行，上位机不额外下降。取回不使用物料识别，固定取放程序自身的升降由下位机负责。底盘在逐件取放期间不平移；入站水平纠偏结束后先停车并用新鲜里程记录P0，环2校准不覆盖P0；收拢完成后默认按二维里程返回P0；比赛可请求原地停稳交接导航。两种模式都确认停车后才完成阶段。两次纠偏超时跳过并报告 `TIMEOUT_SKIPPED`，不能伪报达标。

每次切换到1/3号环，若已保存对应机构标定点，先进入 `PREPOSITION → STOP_PREPOSITION`，只按实际伸缩脉冲和机构角度运动，不要求相机看见下一圆环。例如顺序213，放完1号后即前往3号标定点，到位停稳后才进入 `ALIGN_RING` 找3号环并精修；此时目标仍不可见则等待。2号使用本次工作位记录的位置；未保存对应机构标定点时沿用视觉搜索。8086模拟/真实取放与正式比赛共用此顺序。

图像坐标左上原点、右/下为正。高位底盘校准使用 `vx=-PID(eu), vy=+PID(ev)`；放置使用 `yaw_wz=-PID(eu), fb_rpm=-PID(ev)`，底盘全零。右侧圆心偏高时斜率负，底盘给正角速度纠偏，不追踪绝对yaw目标。

放置达标要求连续4张不同的新帧同时满足中心欧氏容差、旋转小输出、伸缩小输出；记录在机构停止后的新鲜反馈到达时、固定放置开始前完成。取回前后位置按有符号脉冲误差闭环，角度按rad闭环。机构为有限转角，直接比较记录的绝对角，不做多圈展开。

入站底盘环2对准及每件机构圆环PID分别按共用配置限时（当前均3秒），由 `base_servo.timeout_s`、`gantry_servo.timeout_s` 设置。每次从首次看到当前目标并开始对准计时，预定位不占用额度，误差进出容差圈或短暂丢目标不重置。PID一旦超时即停止纠偏并继续本件流程，即使目标此时丢失或没有新的识别帧；底盘仍须确认停车，粗加工机构仍须停稳后记录实际位置。尚未见过目标、未启动PID时仍等待。超时原因明确报告视觉对准未确认达标，不作为PID收敛结果。

预定位及取回的旋转/伸缩PID使用 `gantry_motion.timeout_s`，超时发零速，停稳后分别继续视觉精修或固定夹取。停止复核不得因位置超差重新启动已经超时的取回PID。P0返回PID使用 `exit_motion.timeout_s`，超时后原地停车并继续交接，结果报告 `reference_return_timed_out=true`、`reference_restored=false`，mission接受该结果；不伪报已回P0。8086沿用相同控制逻辑。

## 协议和配置

采用[上位机通信协议v2.0（正文v0.8）](../../../上位机通信协议v2.0.md)，正式动作号为：

| 参数 | cmd | 动作 |
|---|---|---|
| work_pose_cmd | 3 | WORK高位工作位置 |
| place_cmd | 6 | PLACE_TEMP_1粗加工/暂存第一层共用放置，完成后回高位 |
| pick_cmd | 5 | PICK_ROUGH粗加工夹取/入仓，完成后回高位 |
| stow_cmd | 2 | INIT行驶收拢 |

0为STOP/ABORT，1为HOME，均不冒充收拢。v0.6起JOG后位置回读有效即可执行固定动作，不强制HOME；ACK result=5表示位置反馈无效。v0.7的 `fb_pulses` 为int32相对回零零点脉冲，`yaw_rad` 为单圈绝对角。记录、返回、预定位和保存机构初值时须有 `pos_flags.bit0`，不以UD有效或bit2已标定为额外门槛。

[coarse.yaml](config/coarse.yaml)保存区域业务参数，通过 `motion_config` 引用唯一共用运动源 [work_motion.yaml](config/work_motion.yaml)。后者的 `angle_alignment/base_servo/gantry_servo/gantry_motion` 同时用于粗加工和暂存，网页任一区域保存都会更新两区。`gantry_motion` 包括旋转/脉冲PID、限速、容差与超时，同时控制两区前往1/3号预设点和粗加工取回；`gantry_presets.positions` 只保存标定目标位置，取回目标仍是此前记录的实际放料位置。P0返程 `exit_motion` 仍各区独立。

[coarse.yaml](config/coarse.yaml)为业务YAML，通过节点字符串参数 `config` 读取，不直接用作ROS `--params-file`。当前使用较小非空初值，尚未实车标定：

- `arm`：各固定动作号；粗加工不再配置或执行额外定时下降。
- `vision.target_uv_px`：工作位底盘环2/机构放置共用准心，`[]`默认原图中心。
- `angle_alignment`：斜率PID、容差与最多3秒时限。
- `base_servo`：仅初始2号环对准的底盘参数，`timeout_s` 当前3秒。
- `gantry_servo`：放置的像素误差PID、rad/s与RPM限幅、小输出门槛；达标帧数固定4，`timeout_s` 当前3秒，超时继续。
- `ring_search`：找1号时向左、找3号时向右的小幅旋转幅值与时长，之后停止等待；2号不可见则停止等待。
- `gantry_motion`：取回的实际脉冲位置/机构角度PID、各自容差和总返回时限。`length_tolerance_pulses`为脉冲容差，`length_pid`输入脉冲误差、输出RPM。现有数值作为脉冲域调试初值，保留RPM限幅，不代表由旧毫米参数换算得到；无需知道丝杆导程即可记录和返回同一读数。
- `exit_motion`：收拢后底盘移动的限速、比例增益、到位容差和故障时限。
- `execution`：50Hz控制、停止交接时间、反馈有效期与受理时限。

返程参数为 `coarse.yaml` 的 `exit_motion`：两轴限速各0.03m/s、比例增益1、欧氏位置容差1mm、超时15秒。世界坐标误差按当前yaw转换为车体vx/vy，wz=0；到位后等待新鲜低速反馈确认停车。入站纠偏超时仍按原规则继续，先停稳记录P0。原 `camera_car_offset` 配置已移除，Action同名字段仅保留兼容且忽略。

正常图像下尚未启动PID且没有目标时持续等待。PID超时继续；图像断流、反馈过期、控制许可改变、固定动作拒绝/执行超时或结果未知仍停止报错。固定动作的ACK只代表受理，需等待完成；JOG停止并确认idle后才提交固定动作。持续JOG与固定动作均复用唯一桥接，不能另开串口。

## 构建与启动

```bash
source /opt/ros/humble/setup.bash
cd /root/工创/ros2_ws
colcon build --symlink-install --packages-select agv_interfaces agv_planning agv_cboard agv_mission agv_perception
source /root/工创/ros2_ws/install/setup.bash
```

正式比赛按现有启动流程启动唯一C板桥接、视觉宿主和mission。视觉宿主入口：

```bash
ros2 launch agv_perception coarse.launch.py config:=/root/工创/ros2_ws/src/agv_perception/config/coarse.yaml
```

启动不自动发运动指令；mission在阶段6、12派发任务并仲裁底盘最终速度。视觉通过 `/vision/coarse/control` 提交底盘控制，不绕开mission直接发布 `/C/controll`。整场运行仍要求原料、暂存等区域各自就绪，粗加工低速初值不替代其他区域的现场标定。

机构接口为 `/C/arm_command` 固定动作、`/C/arm_jog` 定时升降、`/C/arm_servo` 持续旋转/伸缩，以及 `/C/arm_execution` 执行反馈。实际机构位置来自 `/C/status`，不是底盘里程的yaw。

## 网页调试

见[8086工具说明](../../../tools/coarse_debug/README.md)。退出正式节点和其他相机工具后：

```bash
cd /root/工创
./tools/coarse_debug/start.sh
```

打开 `http://小车IP:8086`，选择“粗加工区”，输入213等顺序。会真实驱动底盘、工作位、旋转/伸缩、工作位恢复、最终收拢和返回P0；只把放置8、夹取5分别替换为停车1秒。模拟放料后直接前往下一标定点，末件后直接按记录取回；模拟取料后仍真实执行WORK恢复高位。真实放料8内部的自动回高位由下位机负责。网页不自动进入下一个比赛阶段。

页面位置表显示每环记录的脉冲/rad，独立显示底盘参考位置。停止后可编辑保存；共用运动组影响粗加工与暂存，区域专用参数同步各自正式配置，网页下一次开始生效，正式节点需重启。返程由各区域 `exit_motion` 配置控制，网页不再读取固定偏置距离。停止/网页失联时中止任务，不自动收拢或续跑。

## 结果与验证

成功结果包含按输入顺序的三条PLACE、三条PICK、最终收拢、离站停车及控制权释放。`final_alignment_outcome`区分 `ALIGNED` 与 `TIMEOUT_SKIPPED`。`reference_restored` 表示收纳后已返回P0并停车，只有返程成功才为true；末次视觉纠偏是否达标由 `final_alignment_outcome` 独立报告。mission允许角度超时跳过，但仍按本次请求检查离站交接。

比赛默认 `merge_exit_navigation=true`，请求 `defer_reference_return=true` 时，收纳后进入 `STOP_EXIT`，保持零速并等待新鲜里程连续低速达 `execution.zero_settle_time_s`，报告 `reference_return_deferred=true, reference_restored=false`。导航保留固定系目标，直接从实际位置驶向首段目标，吸收作业偏移；不重复加位移、不跳过后续拐点。原地停稳超时使用 `exit_motion.timeout_s`；停止、失败不报告交接就绪。关闭比赛开关或独立调用Action（默认false）仍返回P0；8086调试保持原流程。

```bash
cd /root/工创
PYTHONPATH=ros2_ws/src/agv_perception python3 -m unittest discover -s ros2_ws/src/agv_perception/test -p test_coarse_control.py -v
PYTHONPATH=tools/coarse_debug:ros2_ws/src/agv_perception python3 -m unittest discover -s tools/coarse_debug/tests -p test_task.py -v
node tools/coarse_debug/tests/test_web.js
```

真实ROS/PTY测试仅使用软件场景和模拟控制板，不连接硬件；执行方法和验证结果见[新版实施记录](../../../docs/主相机视觉方案/15_粗加工区龙门架实施记录.md)。现场仍需确认方向、工作位共用准心、取放程序起始姿态、机构返回误差及两批实车效果。
