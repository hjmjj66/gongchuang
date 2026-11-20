# 暂存区龙门架执行与标定

运动控制统一读取 [work_motion.yaml](config/work_motion.yaml)，由 `storage.yaml` 的 `motion_config` 引用。水平纠偏、底盘对2号环、机构视觉对准、按角度/脉冲前往预设点四组均与粗加工共用；最后一组也用于粗加工记录位置取回。网页在任一区域修改整组PID、限速、容差及超时均影响两区。准心、标定目标位置、搜索、动作号及P0返程仍按原区域配置；首次与第二次的视觉准心继续分开。

首次纠偏及2号环定位保留全部圆环的几何精定位，按序 `ALIGN_RING` 时仅对当前环找圆，其余YOLO框使用框中心并标记 `NOT_TARGET`。第二次扫描第一层颜色保留原有定位；逐件 `ALIGN_MATERIAL` 仅精定位当前颜色，其余颜色使用框中心。正式流程、8086作业/追踪均采用此策略；切换目标时等待与当前目标匹配的观测，目标找圆失败仍按原有回退配置处理。

2026-11-15：正式阶段8/14和8086网页已按[新版指导计划](../../../docs/主相机视觉方案/14_暂存区龙门架视觉伺服改造指导计划.md)接入高位伸缩、旋转及首次离站补偿。详见[实施记录](../../../docs/主相机视觉方案/16_暂存区龙门架实施记录.md)。以下参数仍须现场标定，软件场景不证明实车精度。

## 两次流程

第一次 `STORAGE_PLACE`：底盘停车 → WORK(3)高位 → 圆环斜率水平纠偏（最多3秒，超时继续） → 停稳记录P0 → 底盘平移将环2对准共用准心并确认停车 → 锁定底盘 → 按第一批位置顺序逐件「高位机构对准 → 4张新帧达标 → 停轴交接 → 第一层放置(6)，程序自动回统一高位」 → INIT(2)收纳并等待完成 → 返回P0并停车，或按请求原地停稳交接导航。环2校准不算一件放置；无取回或末次纠偏。

第二次 `STORAGE_STACK`：整批底盘零速，无角度纠偏及平移校准 → WORK(3) → 机构旋转扫描确认第一层三色 → 实际WORK(3)恢复统一高位 → 按第二批颜色顺序逐件「当前同色物料高位机构对准 → 4张新帧达标 → 停轴交接 → 第二层叠放(7)，程序自动回统一高位」 → INIT(2)收纳。没有二维返回。

例 `156+123+516+231`：第一层映射 `{1:1,5:2,6:3}`，第二批颜色5→1→6、记录位置2→1→3。第四组231不作为暂存码垛顺序。第一层关系只指导搜索和步骤元数据；精对准用当前有效图像，不使用扫描历史像素驱动。

相机随夹爪伸缩及导轨旋转。`eu=u-u_ref, ev=v-v_ref`，机构 `yaw_wz=-PID(eu)`（rad/s）、`fb_rpm=-PID(ev)`（RPM），视觉期间升降0。两轴同时伺服。连续4张不同新帧均满足欧氏像素误差容差、正常限幅后的旋转/伸缩小输出门槛，才关闭连续流并直接交接高位固定放置程序；不额外做视觉停车复核。

P0取入站水平纠偏结束后、环2平移前的停稳坐标，纠偏超时继续时也先停车再记录。首次收纳完成后按当前yaw将世界坐标误差转换为vx/vy返回P0，wz=0；到位后等新鲜低速反馈才结束。`storage.yaml` 的 `exit_motion` 控制返程：两轴限速各0.03m/s、增益1、欧氏容差1mm、超时15秒。默认返回P0成功时 `reference_restored=true, reference_return_deferred=false`；比赛默认请求 `defer_reference_return=true`，收纳后进入 `STOP_EXIT`，保持零速并用新鲜里程连续低速确认停车，报告 `reference_return_deferred=true, reference_restored=false`，由导航首段目标减当前实际位置吸收偏移。原地停车使用 `execution.zero_settle_time_s` 和 `exit_motion.timeout_s`；保留导航固定系基准，不额外加补偿。第二次不返回，两个结果字段均为false，且不接受延期返回请求。关闭mission的 `merge_exit_navigation` 则恢复实际回P0；8086网页默认仍实际回P0。不再读取固定偏置距离。两次暂存不再含 `lower_speed_rpm/lower_duration_s`。

首次底盘环2对准由 `base_servo.timeout_s` 限时；两次机构圆环/物料对准均由 `gantry_servo.timeout_s` 限时，当前均3秒。从首次看到当前目标并开始PID起计时，预定位不计入；误差进出容差圈或丢目标不重置，换件后重新计时。超时停止纠偏并继续当前件放置/码垛，不再等待目标重现；底盘仍须确认停车。超时原因明确标注视觉对准未确认达标。尚未见过目标、未启动PID时仍等待。

机构标定点预定位PID超时停轴后继续视觉对准。首次离站的返回P0 PID超时则原地停稳后继续交接，报告 `reference_return_timed_out=true` 而非伪报已回P0；mission接受该结果。反馈、相机、控制许可及固定动作故障仍按故障处理。

## 寻找与等待

每色连续确认帧数默认3，三色可跨视野累计，但三色未齐不得开始放置。扫描从已知WORK姿态探索未确认的左右位置，每侧有限探索一次；以累计扫描转动确定方向，避免按任意姿态下编号大小判断左右。走到搜索边界仍未齐即全零等待，不循环扫、不自动回位恢复。扫描成功必须实际WORK后重新采图。

逐件优先前往已保存标定点；没有对应点位时位置1左转、3右转、位置2等待，只做一次有限低速搜索。目标丢失立即停轴、清计数，PID限时内一张新帧重现即恢复；限时到则继续放置/码垛。尚未启动PID的目标搜索可一直等。视觉等待沿用同一机构流发送全零速度，进入固定动作时才关闭流并交接。保留取消、反馈、许可和页面心跳检查。图像过期或相机断流、固定动作拒绝/执行超时/结果未知、控制权异常仍停止报错。

## 参数

[storage.yaml](config/storage.yaml) 是业务YAML，由公共节点的 `storage_config` 参数读取。正式未标定值保留null，拒绝执行；常规网页调参不会替代正式标定，8086参数保存同步区域参数及共用运动配置，标定保存准心或机构目标位置。

| 分组 | 内容 |
|---|---|
| `arm` | WORK=3、第一层PLACE=6、第二层STACK=7、INIT=2；不发送额外下降 |
| `vision` | 首次圆环 `target_uv_px` 与第二次物料 `stack_target_uv_px` 分开；空数组为实际图像中心；图像有效期与相机启动时限 |
| `angle_alignment` | 首次斜率PID、容差、连续帧、底盘角速度；目标0，最多3秒 |
| `base_servo` | 首次环2校准两轴PID、底盘m/s限速、像素及停车容差；`timeout_s` 当前3秒 |
| `gantry_servo` | 两次共用旋转/伸缩PID、rad/s和RPM限幅、中心及小输出门槛；固定4帧；`timeout_s` 当前3秒，圆环与物料对准均超时继续 |
| `gantry_motion` | 两区前往预设点及粗加工取回共用旋转/脉冲PID、限速、容差与超时 |
| `gantry_presets.positions` | 已标定1/3号目标角度与脉冲位置，不含运动增益 |
| `ring_search` | 机构搜索角速度、每侧探索时长、每色扫描连续确认帧数 |
| `exit_motion` | 默认回P0的限速、增益、容差及时限；延期返回模式复用timeout_s作为原地停车时限 |
| `execution` | 固定50Hz、停车确认、反馈/许可有效期、机构服务受理时限 |

删除旧 `xy_return/place_servo/stack_servo/stack_work_pose_cmd` 和缺目标业务总时限。两次共用高位工作高度（cmd3）和伺服参数；圆环与物料准心分别标定，层高由cmd6/7区分，仍须现场验证。8086“标定模式”可把圆环点同时写入正式粗加工/暂存首次配置，并把第二次物料点独立写入正式暂存配置；下次启动正式节点生效。旧YAML省略物料准心时沿用旧圆环准心。

粗加工/暂存共用 `vision.ring_config/material_config/model_runtime_path/max_frame_age_s/camera_start_timeout_s`，及 `execution.control_rate_hz/lease_timeout_s`。只运行一个公共 `coarse_node`，共用一份 `/vision` 运行、停止、复位及跨区域互斥接口。

## 启动与调度

```bash
source /opt/ros/humble/setup.bash
cd /root/工创/ros2_ws
colcon build --symlink-install --packages-select agv_interfaces agv_planning agv_cboard agv_mission agv_perception
source /root/工创/ros2_ws/install/setup.bash
ros2 launch agv_cboard cboard.launch.py
```

另一个加载相同环境的终端启动唯一视觉执行端：

```bash
ros2 launch agv_perception coarse.launch.py \
  config:=/root/工创/ros2_ws/src/agv_perception/config/coarse.yaml \
  storage_config:=/root/工创/ros2_ws/src/agv_perception/config/storage.yaml \
  raw_config:=/root/工创/ros2_ws/src/agv_perception/config/raw.yaml
```

比赛由mission在阶段8/14调用 `/vision/storage/run_batch`。底盘速度发 `/vision/storage/control`，经mission仲裁到 `/C/controll`。机构复用 `/C/arm_servo`、`/C/arm_command` 及同一串口；桥接保证连续流停轴、新鲜idle与固定动作之间的交接。点动后不插入HOME，cmd6/7直接从高位工作点起步；ACK只是受理，完成须本次动作忙→闲。

首批返回三项PLACE及布局，第二批三项STACK；`visually_verified=false`，两次 `final_alignment_outcome=NOT_RUN`。调度按完整三步/首批映射、释放、机构空闲、收纳和本次离站请求验收。最终cmd2完成、首次返回P0或原地停车交接、机构流已释放且底盘零速后才报告可行驶；下一路线从实际离站位置启动。

网页见[8086工具说明](../../../tools/coarse_debug/README.md)：工作位、机构伺服、模拟后的工作位恢复、最终收纳及首次离站补偿实际执行；只把放置/码垛替换为全零至少1秒。停止/取消、失联和退出清理底盘、机构及ABORT，不自动收纳或返程。

## 软件验证与现场次序

```bash
cd /root/工创
PYTHONPATH=ros2_ws/src/agv_perception python3 -m unittest discover \
  -s ros2_ws/src/agv_perception/test -p 'test_*control.py' -v
```

已加载ROS及工作空间后，软件伪串口联测：

```bash
OPENBLAS_NUM_THREADS=1 ROS_LOG_DIR=/tmp/agv_storage_pty_tests ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=161 \
  AGV_STORAGE_ROS_TEST=1 PYTHONPATH=ros2_ws/src/agv_perception/test:$PYTHONPATH \
  python3 -m unittest test_storage_ros_pty -v
```

测试使用软件生成观测和PTY，下位机收到的底盘/机构速度改变场景，不打开真实相机、真实串口或车辆。现场先核对3/6/7/2动作表和高位衔接，再低速验证图像方向、统一准心、工作位视野、离站补偿方向/距离与共用层高配置。按单件→三件平放→部分视野扫描→单件/三件码垛→收纳及下一阶段逐步验收。
