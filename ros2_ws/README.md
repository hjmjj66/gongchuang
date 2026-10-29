# ROS2 工作空间

专用调试工具保存在项目根目录 `tools/`，浏览器实车自由行走与比赛路线演练见 [工具说明](../tools/route_debug/README.md)。该入口独立启动调试控制节点，不能与正式mission同时运行。

- 比赛调度：`agv_mission`，已实现整批调度、开始/暂停/复位、阶段日志及统一速度转发，见 [比赛调度 README](src/agv_mission/README.md)。
- 屏幕通信：`agv_screen`，人工禁行与雷达合并、屏幕选择起终点和一键启动、显示本轮扫码结果；[启动与操作说明](src/agv_screen/README.md)。`bash /root/工创/tools/start_screen_mission.sh` 启动调度和屏幕两个节点，硬件执行端仍分别启动；第四阶段成功次数待真实视觉结果接入。
- 扫码节点：`agv_perception`，说明见下文。
- 路径规划节点：`agv_planning`，完整构建、配置与调试说明见 [路径规划 README](src/agv_planning/README.md)。
- C板底盘通信：`agv_cboard`，USB CDC 速度下发、里程反馈、控制权及断流停车，见 [C板通信 README](src/agv_cboard/README.md)。实车接线、方向和停车效果仍需现场验收。
- 路径规划消息与服务：`agv_interfaces`。mission只发目标，规划节点内部确定起点和当前朝向；`/route/path` 输出调试路线和行动链。规划节点读取 `/C/odometry`，通过 `/route/control` 返回速度及执行状态，mission校验后统一发布 `/C/controll`，供C板通信订阅。

新增的比赛消息、Action及路线运行服务需要重新构建相关包。真实主相机区域任务、雷达识别与C板机构通信仍待接入；底盘桥接已实现。

开发测试、临时文件、旧日志、性能检查脚本及纯软件模拟入口已清理。`src`保留比赛源码、接口、配置和启动文件；`build`与`install`用于当前ROS2构建及运行，使用符号链接安装时不能单独删除`build`。硬件资料和比赛规则文件保留在工作空间外层。

## USB 相机二维码扫码节点

面向 RDK X5 / Ubuntu 22.04 / ROS2 Humble。源码在 `src/agv_perception`。

节点启动即扫码，不需要 Action、Service 或其他启动消息。连续 5 张不同的新处理帧解出完全相同的文本后，发布一次结果并关闭扫码相机，节点保留运行。解码算法实现在 `src/agv_perception/agv_perception/qr_decoder.py`：灰度化、Otsu 反向二值化、21×21 闭运算、轮廓/凸包四边形提取、500×500 透视矫正、ZBar 解码。

## 构建和启动

本机已安装 ROS2 Humble、OpenCV、NumPy。方法三额外需要的依赖：

```bash
sudo apt-get install python3-pyzbar libzbar0
```

构建：

```bash
source /opt/ros/humble/setup.bash
cd /root/工创/ros2_ws
colcon build --symlink-install --packages-select agv_perception
source /root/工创/ros2_ws/install/setup.bash
```

启动（明确加载源码 YAML，修改配置后无需重新构建）：

```bash
ros2 launch agv_perception qr.launch.py \
  config:=/root/工创/ros2_ws/src/agv_perception/config/qr.yaml
```

每个新终端都需要先加载 ROS2 和本工作空间：

```bash
source /opt/ros/humble/setup.bash
source /root/工创/ros2_ws/install/setup.bash
```

查看扫码结果及坐标（分别在两个终端运行）：

```bash
ros2 topic echo /qr/result
ros2 topic echo /qr/center
```

只有连续 5 次结果一致才确认成功。文本变化从 1 重新计数；漏检、解码异常、相机断流或帧过期清零；重复读取同一帧不增加计数。确认后仅发布一次文本和第 5 次检测的中心坐标，取消扫码定时器、释放相机并关闭预览。再次扫码需重启扫码节点；按 **Ctrl+C** 结束节点进程。

## 输出约定

| 话题 | 消息类型 | 内容 |
| --- | --- | --- |
| `/qr/result` | `std_msgs/msg/String` | 连续 5 次一致后发布一次原始文本，先尝试 UTF-8，再尝试 GBK；不限比赛格式 |
| `/qr/center` | `geometry_msgs/msg/PointStamped` | 判稳时最后一张图像中的二维码中心，只发布一次；`x`、`y` 为浮点像素坐标，`z=0` |

坐标原点为原图左上角，X 向右，Y 向下。中心取解码得到的二维码四边形对角线交点，利用透视变换的逆变换映射回原图；不是 500×500 矫正图坐标，也不是 A4 纸/外层轮廓中心。`header.stamp` 为读取该帧后的 ROS 时间，`header.frame_id` 是相机标识；这些数值是像素，不能直接作为米制 TF 坐标。

同一画面存在多个可解码二维码时，只发布映射回原图后面积最大的一个。检测阈值、闭运算和透视尺寸沿用原方法三：面积不足整帧 1% 的轮廓会被忽略，实际识别距离和效果需在相机安装位置实测。

尚未连续确认 5 次、未解码到二维码、读帧失败、重复消费同一帧或帧已过期时，两个业务话题均**不产生新消息**，不发布空字符串、零坐标或布尔消息。话题名称及发布者仍存在；`ros2 topic echo` 已显示的历史文字不会被清空。订阅方如有控制动作，应根据中心点时间戳判断观测是否过期。QoS 为 Reliable、Volatile、KeepLast(1)，晚加入的订阅方不会收到节点缓存的上一次扫码结果。ROS2 自身的 `/rosout`、`/parameter_events` 属于框架话题。

本版不拆分独立相机节点，也不实现总体架构文档中的 `/qr/read` Action；当前行为为“启动即扫码、连续 5 次一致后输出并关闭相机”。扫码节点独占扫码相机，后续如接入统一相机节点需调整图像输入方式。

## 调度器转发结果

数据流：`qr_node → /qr/result → mission_node → /mission/qr_result`，文字消息类型均为 `std_msgs/msg/String`。

先启动调度器并建立 `/qr/result` 订阅，再启动扫码节点。原始话题继续使用 Reliable、Volatile、KeepLast(1)，不回放旧轮二维码。调度器暂存本轮提前收到的首个有效任务码，进入 `READ_TASK_CODE / WAIT_QR` 后冻结任务快照并转发一次；无效任务码不转发。暂停、故障或扫码阶段之后收到的结果不再接受；复位会清除任务快照及暂存，下一轮需重新启动扫码节点获取新结果。

`/mission/qr_result` 使用 Reliable、TransientLocal、KeepLast(1)，调度节点存活时，匹配 QoS 的晚加入订阅者可以收到最近一次已接受的结果：

```bash
ros2 topic echo /mission/qr_result --qos-durability transient_local
```

输入和转发 topic 可在 `src/agv_mission/config/mission.yaml` 中通过 `qr_input_topic`、`qr_result_topic` 修改，两个最终解析的话题名必须不同。转发文本表示最近一次已接受结果；当前轮次与有效性以 `/mission/task_info` 中的 `run_id`、`valid` 为准，复位不会把转发缓存当作新轮扫码输入。

## YAML 配置

文件：`/root/工创/ros2_ws/src/agv_perception/config/qr.yaml`。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `camera_device` | `/dev/v4l/by-id/usb-DH_USB2.0_DH_USB2.0-video-index0` | 本机 `05a3:9230` USB 相机；使用稳定路径，不依赖 `/dev/videoN` 编号 |
| `camera_width` / `camera_height` | `640` / `480` | 请求采集分辨率，启动日志会打印实际分辨率 |
| `camera_fps` | `30.0` | 请求相机帧率，实际帧率受驱动及曝光影响 |
| `camera_buffer_size` | `4` | V4L2 采集缓冲数量，避免单缓冲限制采集吞吐 |
| `camera_dynamic_framerate` | `false` | 是否允许自动曝光主动降低帧率；每次连接设置、断开或正常退出时恢复原值；不支持时警告并继续采集 |
| `scan_rate_hz` | `30.0` | 处理最新帧的目标频率，实际受算法耗时及相机实际帧率限制 |
| `show_preview` | `false` | YAML 布尔值，设为 `true` 显示实时边框、中心和坐标 |
| `result_topic` / `center_topic` | `/qr/result` / `/qr/center` | 输出话题名，也支持 ROS remap |
| `frame_id` | `qr_camera_optical_frame` | 中心点消息中的相机标识 |

例如开启预览：

```yaml
    show_preview: true
```

参数在启动时读取；修改 YAML 后重启节点。预览需要有效桌面显示环境，纯 SSH 环境使用 `false`。预览画面不另建图像话题；判稳后自动关闭预览，相机及处理停止，节点进程使用 Ctrl+C 退出。

相机在独立线程持续读取，只保留最新帧，避免扫码较慢导致视频积压。设备无法打开或取流失败时，节点保留运行并每秒尝试重连，期间不发布结果；日志最多每 5 秒提示一次同类错误。连续 5 次解码一致后释放设备，不再重连或继续采集。

关闭动态帧率会保留自动曝光；光照不足时需检查图像亮度、噪声及运动模糊。四个驱动缓冲让采集和处理重叠，线程仍持续读取并只保存最新一帧；长时间处理落后时仍需实测图像时延。30 Hz 是目标处理速度，不保证复杂画面均能达到；两个业务话题只在连续确认 5 次后发布一次。

2026-10-20 实机优化验证：640×480、关闭预览，稳定窗口相机采集约 29.8 fps，扫码处理约 29～29.6 Hz；优化前为约 20 fps 和 15 Hz。当前视野未成功解码，不能将处理频率视为成功发布频率。历史检查结论统一存放在 [相机检查记录](../docs/相机检查记录/扫码相机提速记录.md)。

若以后增加第二路 USB 相机，检查稳定设备路径再更新 YAML。同型号且无序列号的多路相机可能需要使用 `/dev/v4l/by-path/` 区分物理接口。

## 相机检查

使用实际扫码节点检查设备与输出，将二维码放入相机视野：

```bash
source /opt/ros/humble/setup.bash
source /root/工创/ros2_ws/install/setup.bash
ros2 launch agv_perception qr.launch.py
```

另开终端使用前述 `ros2 topic echo /qr/result` 查看输出；没有二维码时消息数为零属于预期。若设备打开失败，检查YAML设备路径、是否被其他程序占用、是否有video组权限。确认后释放相机；未确认时按Ctrl+C结束检查。

历史实机检查：8 秒内收到 120 帧，实际分辨率 640×480；现场画面未成功解出二维码，两个话题收到的消息均为零。真实二维码在安装距离、角度、光照下的效果及预览窗口仍需现场检查。

2026-10-25 二维反馈迁移：`/C/odometry` 改为会话固定系位置、连续yaw及车体系实际速度；速度统一前vx/左vy。每次进入PC、通信重启、重连和板端重启重建原点，通信健康自动恢复，旧任务必须重新初始化。接口不兼容旧二进制，统一重建 interfaces、cboard、planning、mission。详见 [改造计划](../docs/C板通信与二维里程反馈改造计划.md)。
