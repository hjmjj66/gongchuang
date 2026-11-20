# 主相机圆环识别网页调试

在 X5 上用主 USB 摄像头运行用户训练的 YOLOv8，按编号 1、2、3 分别保留**置信度严格大于 0.75 的最高分框**，再在各自扩展 ROI 内执行传统视觉找圆，输出原图浮点像素坐标。只处理新帧，BPU 推理单路，CPU 几何在独立进程内并行处理三个编号 ROI，标注/编码与识别流水执行。各阶段最多一份待处理数据，编码繁忙时跳过该显示帧，避免网页预览阻塞识别；共享内存传递原始像素；多人打开网页不会重复推理或打开相机。

## 启动

先关闭占用同一主摄像头的拍照工具或其他程序。在项目根目录运行：

```bash
./tools/ring_debug/start.sh
```

电脑浏览器访问 `http://板子IP:8084`；网线连接时通常为 [http://192.168.127.10:8084](http://192.168.127.10:8084)，板端本机可访问 [http://127.0.0.1:8084](http://127.0.0.1:8084)。终端按 Ctrl+C 退出并释放相机。

依赖板端已有的 Python 3、OpenCV、NumPy、PyYAML、X5 BPU 驱动及 `hbm_runtime`。网页使用本地 HTML/CSS/JS 和 Python 标准库 HTTP 服务，不需要 ROS 环境、colcon 构建、Node.js 或外网资源。

当前模型与运行库沿用另一会话已经完成单图验证的部署：

```text
/root/ring_deploy_v1_20261103/
├── models/ring_yolov8n_bayese_640x640_nv12.bin
└── rdk_model_zoo/
```

运行库版本对应提交 `286870ca327260af38a3ca4a669d4ed2815af098`。该目录是外部部署依赖，不复制到项目、不自动下载；换板时要保留这份模型与运行库，并更新 YAML 路径。加载失败会在打开相机前报错退出。

## 网页显示

- 黄色：每编号保留的 YOLO 框、类别 `ring_1/2/3`、置信度和当前状态。
- 青色十字 `B`：原始 YOLO 框中心；洋红色圆点 `C`：成功检测并融合的视觉圆心。两者重合时仍用不同形状区分。
- 绿色圆/椭圆：选中环边组；灰框：实际裁剪的处理 ROI。
- 下方三张结果卡：置信度、两种中心坐标、两中心距离、最终来源、几何失败原因。

坐标始终以 **1280×720 等实际原图**左上角为原点，x 向右、y 向下，单位 px；网页缩小画面不改变坐标。找圆失败时，完整框可按配置退回框中心，显示 `BBOX_ONLY`、来源 `bbox`，**视觉圆心留空**；原框触边且找圆失败时返回 `TRUNCATED`，最终中心无效。选中的最高分框失败后，不会改选同编号的较低分框。

帧率分别统计：

| 指标 | 含义 |
|---|---|
| 相机采集 FPS | 近期采集序号增量 / 软件接收时间增量，包括识别跳过的采集帧 |
| 识别处理 FPS | 近期实际完成的 YOLO + 几何处理帧数 / 时间 |
| 网页显示 FPS | 浏览器近期实际解码显示的新帧数 / 时间；重复读取同一帧不计数 |
| 本帧处理 ms | 对应帧的 YOLO + 几何阶段耗时之和（包含跨进程交接），不包含预览编码、网络和浏览器显示 |

流水线的不同帧同时处理，不能直接用 `1000 / 本帧处理 ms` 推算完成 FPS。页面另显示 YOLO、几何阶段耗时，API 的 `result.preview_ms` 记录标注与编码时间；结果过期时仍显示最近耗时，便于诊断。

帧率初次统计尚无两个样本时显示 0；静态图模式显示“—”。相机请求帧率不代表实际识别速度。结果帧年龄从软件读帧完成时起算，不是曝光时间或相机端到端延迟。

图像与识别结果由 `/api/frame` 在同一响应中成对返回。相机断流、帧或结果超过 `max_frame_age_s`、推理失败时清空中心；若还有新鲜相机帧则展示无标注原图。浏览器连接中断也清空图像和坐标，不沿用旧中心。

## 配置与图片调试

网页与 ROS 调试入口共用 [ring.yaml](../../ros2_ws/src/agv_perception/config/ring.yaml)，参数启动时读取，修改后重启。关键配置：

```yaml
ring_debug_node:
  ros__parameters:
    model_path: /root/ring_deploy_v1_20261103/models/ring_yolov8n_bayese_640x640_nv12.bin
    model_format: rdk_yolov8_bin
    model_runtime_path: /root/ring_deploy_v1_20261103/rdk_model_zoo
    class_labels: ["1", "2", "3"]
    confidence_threshold: 0.75  # 必须严格大于；等于也排除
    nms_threshold: 0.7
    process_rate_hz: 30.0
    opencv_threads: 2
    geometry.method: contour
    geometry.contour_max_side_px: 192
    max_frame_age_s: 0.5
```

`class_labels` 是模型索引 0、1、2 对应的工位编号；当前训练类名为 `ring_1/2/3`。官方运行库负责 BGR → letterbox → NV12 → BPU → DFL 解码、一次类内 NMS 和原图框坐标还原；工具负责每编号最高分选择、ROI 几何与原图圆心还原。默认采用局部自适应阈值提取连续环孔轮廓、直接最小二乘椭圆拟合和原分辨率圆周支持检查。候选图可缩小到最长边 192 px，轮廓点在拟合前按像素中心映射还原，输出仍为原图像素。需要至少一条连续可见环边；只有碎弧、严重遮挡或强透视时可能退化，需要实拍标定。`geometry.method=hough` 可用于离线对比，不自动回退到慢霍夫。

网页预览默认保留 1280 px 原宽，避免额外缩放开销；网络受限时可用 `--preview-width 960` 减少传输量，缩小预览不保证 CPU 编码更快。相机仍采集 1280×720，模型仍使用已部署的 640×640 输入。OpenCV 每算子 2 线程，启动脚本将 BLAS 线程池限制为 1，避免多个阶段争用 CPU。ROS 入口共享几何算法，但没有网页的多阶段进程流水线，因此网页 FPS 不代表 ROS 串行入口 FPS。

指定配置、设备、端口或预览宽度：

```bash
./tools/ring_debug/start.sh --config /absolute/path/ring.yaml --device /dev/video0 --port 8084 --preview-width 960
```

不用摄像头，使用相同真实模型和几何流程调试一张图片：

```bash
./tools/ring_debug/start.sh --image /root/ring_deploy_v1_20261103/images/check_254.jpg
```

图片只处理一次，网页重复展示结果，不将它统计成实时帧率。手动 ROI 的几何调试继续使用[包内 ROS 图片入口](../../ros2_ws/src/agv_perception/README.md)。

## 验证

项目根目录执行；普通测试使用明确的测试框输入和真实 OpenCV 几何、图像编码及本机 HTTP，不假冒已接入模型。Node.js 只用于开发测试。

```bash
PYTHONPATH=ros2_ws/src/agv_perception python3 -m unittest discover -s ros2_ws/src/agv_perception/test -v
PYTHONPATH=tools/ring_debug:ros2_ws/src/agv_perception python3 -m unittest discover -s tools/ring_debug/test -v
node tools/ring_debug/test/test_web.js
```

X5 上可额外启用真实模型回归，对比此前板端框坐标并运行两张背景图：

```bash
AGV_TEST_X5_MODEL=1 PYTHONPATH=tools/ring_debug:ros2_ws/src/agv_perception python3 -m unittest discover -s tools/ring_debug/test -p test_x5.py -v
```

2026-11-04 初版（优化前）板端验证：真实模型图片链路与 HTTP 图像/结果接口跑通；示例图 2、3 号输出视觉圆心，1 号圆组歧义时明确退化为框中心。主相机实测采集约 30 FPS、识别处理约 14 FPS，当前视野无候选；这不是圆环目标场景下的速度或准确率保证。浏览器真实渲染、现场圆环动态场景仍需人工验收。


## 实时优化与开源参考（2026-11-04）

现场故障不是 YOLO 未识别：原始实拍帧三个编号的模型置信度约 0.95/0.99/0.97，BPU 推理约 11 ms。旧几何流程产生 1419 个霍夫候选，总耗时约 3.1 秒，超过 0.5 秒有效期，导致网页丢弃结果并显示 0 FPS。没有通过放宽有效期或降低置信度掩盖问题。

参考以下开源实现，采用适合本圆环结构的连续轮廓拟合、局部阈值及限分辨率候选搜索；没有增加新依赖、重新训练或替换模型：

- [OpenCV fitellipse 示例](https://github.com/opencv/opencv/blob/4.x/samples/cpp/fitellipse.cpp)：轮廓与直接最小二乘椭圆拟合。
- [OpenCV contrib EdgeDrawing 示例](https://github.com/opencv/opencv_contrib/blob/4.x/modules/ximgproc/samples/edge_drawing.py)：边缘段生成圆/椭圆候选的路径；板端没有 ximgproc，因此未引入该依赖。
- [AprilTag 局部阈值实现](https://github.com/AprilRobotics/apriltag/blob/master/apriltag_quad_thresh.c)：参考局部亮暗分割思路；此工具使用 OpenCV 自适应均值阈值，不使用 AprilTag 编码识别算法。

最终板端连续采样（约 15 秒，启动预热后 50 个 API 样本）：采集 29.9～30.1 FPS，**识别完成 27.8～28.4 FPS，中位数 28.15 FPS**。单帧 YOLO + 几何阶段之和中位数 58.9 ms；流水线同时处理不同帧，因此完成速率接近 30 FPS。网页结果帧年龄中位数 148.9 ms、范围 119.2～222.0 ms，均从软件读帧完成起算，不是曝光到浏览器的端到端延迟。50 个样本均为 READY，没有放宽 0.5 秒有效期。

本轮真实视野同时有三个编号候选，2、3 号输出视觉圆心，1 号部分被机械结构遮挡、退化为 BBOX_ONLY，未冒充视觉圆心。不是无目标场景测速；此数据描述本轮场景，不保证所有遮挡、光照与 CPU 负载下固定帧率。预览忙时会跳过显示帧，网页显示 FPS 可以低于上述处理 FPS。

验证通过 46 项 Python 测试（32 项核心、14 项网页/流水线/真实 X5，其中包括共享内存复用与释放、慢编码不阻塞几何、坐标还原及真实模型回归）、前端 Node 测试和 agv_perception 的 colcon 构建。
