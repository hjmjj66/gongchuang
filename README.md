# 工创 AGV

基于 ROS 2 Humble 的工创比赛小车上位机代码，包含路径规划、C 板通信、比赛调度、二维码识别和网页调试工具。

## 目录

- `ros2_ws/src/agv_interfaces`：消息、服务和 Action 接口。
- `ros2_ws/src/agv_planning`：路径规划与路线执行。
- `ros2_ws/src/agv_cboard`：底盘串口通信、反馈与诊断。
- `ros2_ws/src/agv_mission`：比赛流程调度。
- `ros2_ws/src/agv_screen`：屏幕禁行路径、起终点及启动交互、扫码结果展示，见 [屏幕接入说明](ros2_ws/src/agv_screen/README.md)。
- `ros2_ws/src/agv_perception`：二维码相机与识别。
- `tools/route_debug`：网页路线调试工具。
- `docs`：设计方案、实施和检查记录。

## 构建

安装 ROS 2 Humble、colcon 及各包依赖后，在仓库根目录运行：

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

网页调试工具从仓库根目录启动：

```bash
./tools/route_debug/start.sh
```

默认访问 `http://127.0.0.1:8080`。该工具直接控制实车，运行前请按工具说明检查现场和串口配置；不能与正式比赛调度同时控制底盘。

## 文档

- [工作空间说明](ros2_ws/README.md)
- [网页调试工具说明](tools/route_debug/README.md)
- [C 板实车测试操作指南](C板实车测试操作指南.md)
- [项目总体架构与开发指南](ROS2项目总体架构与开发指南.md)
- [上位机通信协议](上位机通信协议(4).md)

仓库保留网页地图所需的赛场截图。构建产物、缓存、本地配置及厂商资料包不纳入版本管理；克隆后需要重新构建。
