# 相机与视觉工具

原料区现流程：沿工作方向前进12 mm（`base_motion.distance_mm`可调）→ 原地等待所需颜色并追踪抓取 → 抓取程序回高位，在当前位置等待下一件 → 三件完成收纳后退回P0；正式比赛可由离区导航吸收偏移。取消初始任意颜色跟随和m0机构返回。详见[原料流程](RAW.md)。

暂存任务已接入同一公共 `coarse_node` 和比赛阶段 8、14：首次水平纠偏后停稳记录P0，环2底盘校准后锁定底盘，第二次底盘全程不动；均在高位直接机构视觉对准及放置，完成三件后收纳；首次再按二维里程返回P0并停车，无末次纠偏。独立配置、启动方式、动作契约和验证见 [STORAGE.md](STORAGE.md)。只启动一份公共视觉执行端，未标定参数仍拒绝对应运动任务。

粗加工在比赛阶段6、12执行：高位水平纠偏（最多3秒）后停稳记录P0，再用底盘对准2号环，保持工作位用机构旋转/伸缩视觉对准，记录实际脉冲位置/机构角度，同序按反馈取回，最后高位纠偏再收拢，按二维里程返回P0并确认停车；取回使用机构反馈。底盘速度仍经mission转发。参数、启动和接口见[COARSE.md](COARSE.md)。粗加工使用待现场标定的小幅非空初值；其他区域仍按各自配置校验。

包内现有扫码入口 `qr_node` 保持原来的配置和行为。圆环部分已完成传统几何核心、编号框处理、图片调试，并接入用户在 X5 上部署的 YOLOv8 BIN。类别 `ring_1/2/3` 对应内部索引 0/1/2，映射为工位编号 1/2/3。

新增[主摄像头圆环网页调试](../../../tools/ring_debug/README.md)：在项目根目录运行 `./tools/ring_debug/start.sh`，电脑访问 `http://板子IP:8084`。页面显示 YOLO 框、类别、置信度、青色框中心、洋红色视觉圆心和原图像素坐标，分别统计采集、处理、网页显示 FPS。无需 ROS 环境或构建。

物料工具已实现 HSV 六色分割、YOLO 顶面（优先霍夫圆心、框中心兜底）、YOLO 整体（直接框中心），新增 `use_jhb_model` 模式：JHB 物料/顶帽配对、复用圆环几何精定位、Lab 六色判别（拒判时保留中心仅供预览）；网页顶部可实时选择“原有识别 / JHB”，无需重启；`--use-jhb-model` 或 YAML 设为 true 决定初始模式。原三条路径通过独立的 `material_backend` 与 `material_model` 两参数选择。网页运行 `./tools/material_debug/start.sh`，访问 `http://板子IP:8085`；图片、ROS、Python 接口、真实模型和验证范围见[物料工具说明](../../../tools/material_debug/README.md)。物料保留同色多实例，中心来源明确。六色 YOLO 置信度达标且检测框与画面有交集即可继续定位，不要求完整入镜，找圆失败时按配置使用原框中心；HSV 截断、JHB 顶帽边距不足时仍不提供中心；JHB 颜色拒判保持 `valid: false/color_id: 0`，不能触发指定颜色追踪和抓取。 新增试验模式 `test_combine`：原六色YOLO只判类别，JHB只定位顶帽中心，按同帧框IoU、中心包含和一对一关系匹配，完全跳过Lab；8085/8086网页均可切换，显示双模型分项耗时，详见[融合模式说明](../../../tools/material_debug/README.md#融合试验模式test_combine)。

## 构建与测试

```bash
source /opt/ros/humble/setup.bash
cd /root/工创/ros2_ws
colcon build --symlink-install --packages-select agv_perception
source /root/工创/ros2_ws/install/setup.bash
python3 -m unittest discover -s /root/工创/ros2_ws/src/agv_perception/test -v
```

## 无模型时调试圆心

使用真实相机图片，手动给出一个工位的完整外圈框。这里的四个坐标是原图中的 `x1,y1,x2,y2`，不是宽高；请替换下面的图片路径和框。

```bash
ros2 run agv_perception ring_debug_node --ros-args \
  --params-file /root/工创/ros2_ws/src/agv_perception/config/ring.yaml \
  -p mode:=geometry_only \
  -p input_image:=/absolute/path/rings.jpg \
  -p 'manual_roi:=[170,100,330,260]' \
  -p manual_ring_id:=2 \
  -p output_dir:=/tmp/ring_debug
```

默认不打开桌面窗口，完成后退出。输出目录包含 `result.json`、`image_annotated.png`，以及实际处理 ROI 的预处理图、Canny 边缘图和候选圆图。候选图中绿色为通过检查的候选，红色为被拒绝的候选；具体支持度和拒绝原因可通过核心返回的 `result.debug` 查看。

`manual_ring_id` 可不填（0），或填 1/2/3 用于标注。所有手动 ROI 结果都明确标记 `identity_source=manual`，不能认为模型已识别该数字。无圆时会按配置返回手动框中心，并显示 `center_source=bbox`，这同样不代表几何精定位成功。

有桌面时增加 `-p show_preview:=true`，按 q 或 Esc 退出。预览在图像副本上绘制，不污染识别输入。

## 正式组合路径与 X5 模型

配置文件为 [ring.yaml](/root/工创/ros2_ws/src/agv_perception/config/ring.yaml)。当前沿用另一会话已部署并验证的用户模型和官方运行库：

```yaml
model_path: /root/ring_deploy_v1_20261103/models/ring_yolov8n_bayese_640x640_nv12.bin
model_format: rdk_yolov8_bin
model_runtime_path: /root/ring_deploy_v1_20261103/rdk_model_zoo
class_labels: ["1", "2", "3"]
confidence_threshold: 0.75
nms_threshold: 0.7
```

配置正确且板端 BPU 可用时，可启动正式入口：

```bash
ros2 launch agv_perception ring_debug.launch.py
```

也可以复制 YAML 后通过 `config:=/absolute/path/ring.yaml` 指定。运行库对应提交 `286870ca327260af38a3ca4a669d4ed2815af098`，执行 letterbox、NV12、BPU 推理、DFL 解码、一次类内 NMS 和原图框还原。该目录是外部部署依赖，换板时要保留并更新路径；工具不自动下载或替换权重。

模型参数为空时报告 `MODEL_NOT_CONFIGURED`；文件、运行库或 BPU 加载失败时报告 `MODEL_LOAD_FAILED`；未支持格式报告 `MODEL_ADAPTER_NOT_IMPLEMENTED`。以上均在打开相机前非零退出，不用假模型或几何退化绕过模型缺失。

模型标注需以完整外圈为框，中心数字用于类别识别。三个工位分别对应类别 1、2、3，而非一个框包含三个工位。适配层将模型索引映射为语义编号；官方运行库负责缩放/补边反变换和模型后处理，输出原图框，不重复执行 NMS。

正式路径为：每编号取置信度严格大于阈值的最高分框（当前为 0.75，等于也排除）→ 扩展并裁剪 ROI → 灰度/可选形态学 → 连续环孔轮廓与椭圆拟合 → 圆心范围、半径、圆周支持与角段覆盖检查 → 同心归并与中心融合 → 原图中心。最高分框几何失败时不切换较低分框。

一个可信圆即可；同心组以边缘支持加权中心。重复半径不重复计数，组内最大圆心距离受限。候选排序还比较圆周与边缘的贴合程度 `edge_alignment`，避免仅在径向容差边界找到边缘的偏移圆占优。多个组依据支持程度、贴合程度、距框中心的距离和少量圆数加分排序；分数差不超过 `group_ambiguity_margin` 则视为几何歧义，按退化规则处理。

几何失败且原框未触及原图边界时，允许使用原检测框中心，来源为 `bbox`；扩展 ROI 的中心不用于退化。原框触边时，可信圆仍可用，但不使用框中心。无框或低于置信度阈值时中心为空，不继承上一帧结果。

默认 `geometry.method=contour`：局部阈值提取连续环包围的孔，候选搜索最长边 192 px，拟合前还原轮廓点，所有边缘支持检查和坐标均在原图尺度进行。`method=hough` 保留用于离线对比，密集虚线下不适合实时处理。椭圆中心用于适度斜拍的视觉定位，强透视下不等于物理圆心的精确投影；至少需要一条连续可见的环边。

几何参数需要用真实固定主相机图片调整。`geometry.preprocess=gray` 为默认，`binary_morphology` 为暗色反二值化后小核开闭运算。所有半径比例以原检测框短边为尺度。

## 核心接口

区域程序可以直接导入核心，把统一采集的 BGR 图像交给工具；工具不打开相机，不控制底盘或机械臂，不判断是否完成放料。

```python
from agv_perception.ring_detector import RingDetector
from agv_perception.ring_model import ModelDetection

# 调用方已经有实际推理框时，可以跳过模型加载直接使用几何组合入口。
boxes = [ModelDetection(label="2", confidence=0.85,
                        bbox_xyxy=(170.0, 100.0, 330.0, 260.0))]
detector = RingDetector()
result = detector.detect_boxes(frame_bgr, boxes, frame_seq=12, stamp=read_time_s)
```

自动组合入口为 `RingDetector(model=adapter).detect(frame, frame_seq, stamp)`，适配对象提供 `infer(frame_bgr)`，返回 `ModelDetection` 列表。用 `load_ring_model(model_path, 'rdk_yolov8_bin', ['1','2','3'], runtime_path, confidence_threshold=0.75, nms_threshold=0.7)` 加载一次，传给核心即可。手动几何入口为 `detect_roi(frame, bbox_xyxy, ring_id=None, frame_seq=0, stamp=0.0)`。核心不依赖 ROS，可独立处理已有图像。

`result.observations` 的自动结果固定按编号 1、2、3 排列，缺失编号也有无效项；手动结果只有指定 ROI 一项。重要字段：

| 字段 | 含义 |
|---|---|
| `ring_id` / `identity_source` | 编号与身份来源，手动调试不冒充 YOLO |
| `valid` / `status` | 当前观测可用性；`BBOX_ONLY` 仍表示估计值，不表示精对准 |
| `center_px` / `center_source` | 原图浮点像素中心，来源为 `circle` / `bbox`；无效时为空 |
| `bbox_center_px` | 原始检测框中心，始终与框对应；最终定位无效时仍可用于调试 |
| `circle_center_px` | 仅真实几何定位成功时提供；框中心退化时为空，不冒充圆心 |
| `yolo_confidence` | 模型类别/检测置信度，手动输入为空，不代表中心精度 |
| `bbox_xyxy` / `roi_xyxy` | 原框与实际扩展裁剪范围 |
| `radii_px` / `circle_support` | 所选圆组半径、支持比例和圆数 |
| `geometry_reason` | 无圆、弱支持、组歧义或 ROI 过小等原因 |

`to_dict()` 提供可 JSON 序列化的结果，不带图像数组；`debug` 保留预处理图、边缘图、候选及选中圆组。帧级 `OK` 仅表示所有预期项都有有效观测，可能包含框中心；最终精度应检查各项 `center_source`。

像素原点在原图左上角，x 向右、y 向下；坐标不是米或机械臂坐标。时间由调用方提供；相机包装使用软件读帧完成时间，不能解释为曝光时间或已测量的端到端延迟。

## 相机调试

将 `input_image` 留空使用相机。相机配置默认 1280×720、30 fps 请求、4 缓冲；设备稳定路径来自历史 C100 检查记录，需现场核对。处理频率与相机帧率分开，不承诺推理能达到请求帧率。网页图像与识别结果按同一帧成对返回；识别失败或结果过期时清空中心，若有新鲜相机帧则显示无标注原图。

`output_dir` 非空时相机输出追加到 `results.jsonl`；设置 `save_every_n_frames` 才抽样保存图片。只处理新序号帧，过期帧不检测，推理结束已过期也不记录为有效结果。无帧、过期帧和过期结果分别报告 `NO_FRAME`、`STALE_FRAME`、`STALE_RESULT`。

独立调试入口独占主相机；将来区域节点集成时由主相机节点统一采集并直接调用核心，不能同时再启动该相机调试进程。

## 比赛与区域调试共用视觉执行方式

正式比赛、8087 全场调试和 8086 区域调试共用 `agv_perception.work_vision.RingVision`（含独立进程的物料识别/圆环几何及异步预览），三区域控制器仍为 `RawTask`、`CoarseTask`、`StorageTask`；调试模式只包装逐件停留替代。原料每色保留最大实例，暂存保留各实例，按当前目标精修。正式入口只适配任务启停，不再维护另一套识别流水线。

三区域整批作业共用 `VisionGate`：工作姿态或识别目标切换后，仅采用当前目标的新鲜图像；冷启动的过期结果先丢弃并保持零速，在 `camera_start_timeout_s` 内等待首张合格图像。建立有效图像流后，超过 `max_frame_age_s` 的结果立即停止所有视觉运动并清除跟随状态，等待及时新帧后继续；连续 `camera_start_timeout_s` 没有有效结果才结束任务。有效画面缺少物料继续停车等待。8087 故障日志增加图像序号、图像年龄、采集/识别帧率及阶段耗时。

更新后执行 `cd /root/工创/ros2_ws && colcon build --packages-select agv_perception --symlink-install`，随后重启比赛或调试入口。无需更改串口协议和现场 PID 参数。
