# 相机与视觉工具

包内现有扫码入口 `qr_node` 保持原来的配置和行为。圆环部分已实现传统几何核心、编号框处理、图片调试及后续模型接入边界；用户尚未训练 YOLO，自动编号完整链路等待真实模型和对应运行时接入。

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

## 正式组合路径与模型占位

配置文件为 [ring.yaml](/root/工创/ros2_ws/src/agv_perception/config/ring.yaml)。模型相关参数保持：

```yaml
model_path: ""
model_format: ""
class_labels: ["1", "2", "3"]
```

此时启动正式入口会以非零退出码报告 `MODEL_NOT_CONFIGURED`，并在打开相机前退出：

```bash
ros2 launch agv_perception ring_debug.launch.py
```

也可以复制 YAML 后通过 `config:=/absolute/path/ring.yaml` 指定。只填入模型路径并不能启用推理：当前尚未选择部署格式和运行时，文件存在且参数非空时会报告 `MODEL_ADAPTER_NOT_IMPLEMENTED`。收到真实模型后再补该适配层；不包含示例权重、不下载模型、不使用第四份开源的 Maix 模型。

模型标注需以完整外圈为框，中心数字用于类别识别。三个工位分别对应类别 1、2、3，而非一个框包含三个工位。未来适配层负责内部类别索引映射、缩放/补边反变换以及所需模型后处理，输出原图框；已有 NMS 不重复执行。

正式路径为：每编号取达到置信度阈值的最高分框 → 扩展并裁剪 ROI → 灰度/可选形态学 → 标准霍夫圆 → 圆心范围、半径、圆周支持与角段覆盖检查 → 同心归并与中心融合 → 原图中心。最高分框几何失败时不切换较低分框。

一个可信圆即可；同心组以边缘支持加权中心。重复半径不重复计数，组内最大圆心距离受限。候选排序还比较圆周与边缘的贴合程度 `edge_alignment`，避免仅在径向容差边界找到边缘的偏移圆占优。多个组依据支持程度、贴合程度、距框中心的距离和少量圆数加分排序；分数差不超过 `group_ambiguity_margin` 则视为几何歧义，按退化规则处理。

几何失败且原框未触及原图边界时，允许使用原检测框中心，来源为 `bbox`；扩展 ROI 的中心不用于退化。原框触边时，可信圆仍可用，但不使用框中心。无框或低于置信度阈值时中心为空，不继承上一帧结果。

几何参数是合成图调试起点，需要用真实固定主相机图片调整。`geometry.preprocess=gray` 为默认，`binary_morphology` 为暗色反二值化后小核开闭运算。所有半径比例以原检测框短边为尺度。

## 核心接口

区域程序可以直接导入核心，把统一采集的 BGR 图像交给工具；工具不打开相机，不控制底盘或机械臂，不判断是否完成放料。

```python
from agv_perception.ring_detector import RingDetector
from agv_perception.ring_model import ModelDetection

# 以下是外部真实模型适配层应给出的数据格式示意，不是已接入的推理。
boxes = [ModelDetection(label="2", confidence=0.85,
                        bbox_xyxy=(170.0, 100.0, 330.0, 260.0))]
detector = RingDetector()
result = detector.detect_boxes(frame_bgr, boxes, frame_seq=12, stamp=read_time_s)
```

自动组合入口为 `RingDetector(model=adapter).detect(frame, frame_seq, stamp)`，未来适配对象提供 `infer(frame_bgr)`，返回 `ModelDetection` 列表。手动几何入口为 `detect_roi(frame, bbox_xyxy, ring_id=None, frame_seq=0, stamp=0.0)`。核心不依赖 ROS，可独立处理已有图像。

`result.observations` 的自动结果固定按编号 1、2、3 排列，缺失编号也有无效项；手动结果只有指定 ROI 一项。重要字段：

| 字段 | 含义 |
|---|---|
| `ring_id` / `identity_source` | 编号与身份来源，手动调试不冒充 YOLO |
| `valid` / `status` | 当前观测可用性；`BBOX_ONLY` 仍表示估计值，不表示精对准 |
| `center_px` / `center_source` | 原图浮点像素中心，来源为 `circle` / `bbox`；无效时为空 |
| `yolo_confidence` | 模型类别/检测置信度，手动输入为空，不代表中心精度 |
| `bbox_xyxy` / `roi_xyxy` | 原框与实际扩展裁剪范围 |
| `radii_px` / `circle_support` | 所选圆组半径、支持比例和圆数 |
| `geometry_reason` | 无圆、弱支持、组歧义或 ROI 过小等原因 |

`to_dict()` 提供可 JSON 序列化的结果，不带图像数组；`debug` 保留预处理图、边缘图、候选及选中圆组。帧级 `OK` 仅表示所有预期项都有有效观测，可能包含框中心；最终精度应检查各项 `center_source`。

像素原点在原图左上角，x 向右、y 向下；坐标不是米或机械臂坐标。时间由调用方提供；相机包装使用软件读帧完成时间，不能解释为曝光时间或已测量的端到端延迟。

## 模型到位后的相机调试

未来接入真实模型后，可将 `input_image` 留空使用相机。相机配置默认 1280×720、30 fps 请求、4 缓冲；设备稳定路径来自历史 C100 检查记录，需现场核对。处理频率与相机帧率分开，不承诺推理能达到请求帧率。

`output_dir` 非空时相机输出追加到 `results.jsonl`；设置 `save_every_n_frames` 才抽样保存图片。只处理新序号帧，过期帧不检测，推理结束已过期也不记录为有效结果。无帧、过期帧和过期结果分别报告 `NO_FRAME`、`STALE_FRAME`、`STALE_RESULT`。

独立调试入口独占主相机；将来区域节点集成时由主相机节点统一采集并直接调用核心，不能同时再启动该相机调试进程。
