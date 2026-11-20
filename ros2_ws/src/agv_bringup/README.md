# 整套比赛启动

统一启动 mission、触屏通信、C 板桥接、正式规划器、三区域公共视觉执行端和扫码节点。
启动后保持 `WAIT_SETUP`，由触屏提交本轮禁行路径和起终点，至少两秒且全部必需执行端
就绪后才开始。没有浏览器心跳依赖。不会自动重启故障节点或续跑比赛。

## 构建一次

```bash
source /opt/ros/humble/setup.bash
cd /root/工创/ros2_ws
colcon build --symlink-install --packages-up-to agv_bringup
```

## 一键运行

先退出原有正式节点、底盘/区域网页工具及占用相机或屏幕串口的程序。在 X5 上运行：

```bash
bash /root/工创/tools/start_competition.sh
```

默认前台运行，日志直接显示在当前终端，按 **Ctrl+C** 统一退出整套比赛进程。
运行期间保持终端和 SSH 连接。先检查日志中的 `WAIT_SETUP` 和配置/设备错误，再在触屏操作。
节点全部就绪才允许运动，未标定配置仍拒绝启动比赛；本入口不改变标定值，不使用网页参数替代正式参数。
Ctrl+C 不是物理急停，也不自动复位车辆、物料或机构。

如需后台运行，显式使用 `start`，之后用 `stop` 请求退出：

```bash
bash /root/工创/tools/start_competition.sh start
bash /root/工创/tools/start_competition.sh status
bash /root/工创/tools/start_competition.sh logs
bash /root/工创/tools/start_competition.sh stop
```

后台日志在仓库 `ros2_ws/log/competition/competition.log`，`logs` 显示最后80行。
`status` 只表示 launch 进程存活，不表示硬件 READY。前台和后台启动均使用内核文件锁防止重复运行；
`stop` 只向确认持有本入口运行锁的进程发送 SIGINT，由 ROS launch 统一退出子进程。

启动前检查已有控制节点、服务端和底盘发布者，冲突时拒绝启动，不结束其他进程。
六个比赛子进程中任一个退出都会关闭整套 launch，不配置 respawn。
扫码仍是连续五次一致后仅发布一次并释放相机；统一入口开启 `wait_for_mission`，
等收到调度状态且扫码发布端发现订阅后才消费扫码帧，避免启动顺序丢结果。
下一轮人工恢复现场后，建议停止并重新启动整套入口，以重新准备扫码；本次不增加扫码自动重启机制。

## 雷达开关

唯一开关在 `agv_mission/config/mission.yaml`：

```yaml
mission_node:
  ros__parameters:
    use_lidar: false
```

- `false`：不启动或依赖雷达；出启停区后记录 `SCAN_SKIPPED`，只用本轮人工禁行边发布地图。
  即使未选禁行，也提交本轮空地图；等待规划器确认轮次、版本和内容后才前往二维码区。
- `true`：加载 `agv_obstacles/launch/obstacles.launch.py`，mission 恢复完整雷达运行查询、
  复位、停止、心跳状态监视和扫描依赖，扫描结果与人工禁行取并集。
  当前雷达包仅占位，启用时会在启动硬件进程前报错，不能将占位当作扫描成功。

直接构造 mission 且未提供该参数时保留旧行为（true）；仓库默认 YAML 明确设为 false。
开关在启动时读取，运行中不可修改；修改后重新启动整套进程。

## 配置覆盖与后续部署

脚本默认直接读取各包源码目录的正式 YAML，所有 `*_config` 均可覆盖为绝对路径：
`mission_config`、`screen_config`、`cboard_config`、`planner_config`、
`qr_config`、`raw_config`、`coarse_config`、`storage_config`。

```bash
bash /root/工创/tools/start_competition.sh run mission_config:=/绝对路径/mission.yaml
```

脚本显式绑定源码 YAML，修改现有 YAML 后重启生效，不依赖 install 目录是否生成符号链接；
修改 Python 代码、接口或新增包/文件后仍需构建。统一入口固定正式模式及屏幕控制，规划器使用 REAL 来源，
不启用任意起点调试；不启动任何网页控制器。

也可加载 ROS 环境后直接运行 `ros2 launch agv_bringup competition.launch.py`，
它有相同的节点监督和冲突检查，但属于前台运行，不使用 bash 入口的后台管理锁，
且默认读取各包已安装的配置；未重新构建时应显式传入源码 `*_config` 路径。

当前默认采用 launch 前台启动，通过 Ctrl+C 退出后可重新启动。若以后需要开机进入等待触屏状态，
可让 systemd 管理同一个前台 `run` 命令并保持 `Restart=no`，避免故障自动重开比赛。
本次不安装系统服务、不设置开机自启。
