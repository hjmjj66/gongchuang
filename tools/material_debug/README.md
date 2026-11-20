# 主相机物料识别调试

已实现 HSV 六色、YOLO 顶面、YOLO 整体和 JHB 物料/顶帽＋Lab 四条路径，以及网页、ROS 图片/相机入口和可复用 Python 核心。工具只输出定位参考，不控制底盘或机械臂。

## 启动网页

在项目根目录运行，无需加载 ROS 或构建：

```bash
cd /root/工创
./tools/material_debug/start.sh
```

电脑浏览器访问 `http://板子IP:8085`。运行前关闭占用同一主相机的扫码、圆环、拍照等程序。当前 YAML 设备为历史 C100 稳定路径；若换 USB 口或相机，先检查设备是否存在。

先用部署目录中的原始图片验证模型：

```bash
./tools/material_debug/start.sh --image /root/material_deploy_v1/export/upper_top/test_images/check.jpg
```

可选参数：`--config /absolute/path/material.yaml`、`--device /dev/videoN`、`--port 8085`、`--preview-width 1280`、`--host 0.0.0.0`。`--image` 与 `--device` 不能同时使用。Ctrl+C 退出并释放本次资源。

配置：[material.yaml](/root/工创/ros2_ws/src/agv_perception/config/material.yaml)。网页顶部“识别模式”可实时切换原有识别与 JHB，无需重启；选择仅对当前服务进程生效，不写回 YAML。其他 YAML 参数修改后仍需重启。

```yaml
material_backend: 2        # 1 HSV；2 YOLO
material_model: 1          # YOLO 下：1 顶面；2 整体
```

- 顶面：先找可靠霍夫圆心；失败时默认使用**原检测框中心**。`top.refine_circle: false` 可对比顶面只取框中心的效果，`top.allow_bbox_fallback: false` 则在找圆失败时不输出有效中心。
- 整体：直接取原检测框中心，不执行霍夫和 HSV 精定位。
- HSV：六色分割、轮廓质心、多实例；阈值来自 `zuixin.py`，只是调试起点。实际掩膜高度重叠的跨颜色候选标记 `COLOR_AMBIGUOUS`，不给有效中心。指定颜色过滤发生在歧义检查之后。

六色业务编号：红1、黄2、蓝3、绿4、黑5、浅蓝6。两模型均按显式标签映射编号，允许同色多个实例。六色 YOLO 保留置信度严格大于阈值且与画面有交集的候选，不限制为每色一个，也不要求完整入镜；原框触边仍继续定位，找圆失败时按 `top.allow_bbox_fallback` 使用原框中心。截断后的框中心可能偏移，检测置信度不代表中心精度。

## 融合试验模式：test_combine

网页识别模式新增 `test_combine · 原 YOLO 颜色 + JHB 中心`，也可运行：

```bash
/root/工创/tools/material_debug/start.sh --test-combine
```

或在 `material.yaml` 设置 `test_combine: true`、`use_jhb_model: false`。两开关互斥，均为 false 时恢复原有识别。作业调试网页8086的“物料识别模式”也提供同一选项，停止任务后可切换，用于原料跟随和暂存第二次物料对准。默认配置不变。

两模型处理同一张原图、同一搜索ROI。原六色YOLO只提供类别、置信度和框，按 `material_model` 选择原顶面或整体模型，忽略 `material_backend` 的HSV选择；JHB只检测、配对和定位顶帽，不加载Lab颜色文件，不执行白平衡或Lab判色。最终中心始终来自JHB精修圆心，或按原 `top.allow_bbox_fallback` 设置回退到JHB顶帽框中心，不使用旧YOLO的中心。

匹配使用框IoU（交集/并集，默认 `combine.min_iou: 0.3`）和“JHB帽框中心落在颜色框内”。旧顶面模型与JHB顶帽框比较；旧整体模型与JHB物料框比较。只有双方均唯一对应时才采用旧YOLO颜色，颜色置信度须严格大于原 `confidence_threshold`。无匹配为 `COLOR_UNMATCHED`，存在多个对应为 `COLOR_MATCH_AMBIGUOUS`；这些候选以颜色0保留已取得的中心，仅参与原料初始跟随，不进入指定颜色追踪、抓取或暂存扫描。不会用Lab补判，不跨帧借用旧颜色。

输出中 `bbox_xyxy/yolo_confidence` 属于JHB顶帽，`material_bbox_xyxy` 属于JHB整体，`color_bbox_xyxy/color_confidence/raw_label` 属于旧YOLO，`identity_source: yolo_jhb`。网页标出两边框与中心，并显示原YOLO颜色置信度和匹配状态。耗时包括 `color_yolo`、`jhb_yolo`、`matching`、`geometry`；`yolo` 是两模型推理的耗时之和。8086同时显示识别FPS和采集到结果发布的年龄。

当前环境无BPU设备，验证使用真实JHB ONNX与明确提供的旧YOLO检测输入，不代表双BPU实测速度。X5上可运行现有真实模型测试，新增用例会打印静态图融合FPS与分项平均耗时（非实时相机帧率）：

```bash
cd /root/工创
AGV_TEST_X5_MATERIAL=1 OPENBLAS_NUM_THREADS=1 \
  PYTHONPATH=tools/material_debug:ros2_ws/src/agv_perception \
  python3 -m unittest discover -s tools/material_debug/test -p test_x5.py -v
```

## JHB 模式：use_jhb_model

启动现有网页工具时加开关（默认读取模型包的 X5 浮点输出 `.bin`）：

```bash
cd /root/工创
./tools/material_debug/start.sh --use-jhb-model
```

网页打开后可在顶部“识别模式”选择“原有识别（按启动配置）”或“JHB 模型 · Lab 六色”，选择后立即请求切换。原有识别继续使用启动配置里的 `material_backend/material_model`（HSV、顶面或整体）；JHB 使用 `jhb.*`。切换期间暂停识别、清空旧坐标，加载完成后恢复；静态图片会重新处理一次。模型加载失败时显示原因并继续使用切换前的模式。切换会影响访问此服务的所有浏览器，刷新页面可看到当前生效模式；手动 `geometry_only` 模式禁用切换。

启动参数决定初始模式，也接受 `--use_jhb_model`。或在 `material.yaml` 设置 `use_jhb_model: true`，网页和 ROS 共用；开关优先于 `material_backend/material_model`，`geometry_only` 仍为人工指定颜色的原有手动入口。默认是 `false`，原有三条路径按原配置运行。

不依赖 BPU 的 ONNX 静态图验证：

```bash
./tools/material_debug/start.sh --use-jhb-model \
  --jhb-model-path /root/去年开源/gc_material_model_v218_20261116/detector/material_cap_v218_384x640_9a6d18d0cc69.onnx \
  --image /root/去年开源/gc_material_model_v218_20261116/detector/selftest.jpg
```

ROS 使用同一参数：

```bash
ros2 run agv_perception material_debug_node --ros-args \
  --params-file /root/工创/ros2_ws/src/agv_perception/config/material.yaml \
  -p use_jhb_model:=true
```

流程为：原图缩工作图 → 640×384 letterbox → 两类检测及分类别 NMS → 物料/顶帽一对一配对 → 顶帽 ROI 几何精定位 → 顶帽核心 Lab 六色判别。顶帽分数至少0.7，物料分数至少0.25，顶帽短长边比至少0.8、落在物料框内的面积比例至少0.7，顶帽各边距画面至少1.2工作图像素；均可在 `jhb.*` 调整。配对筛除原因输出在 `rejected_candidates`。

几何直接复用圆环工具的 `RingGeometry`：轮廓/椭圆拟合、边缘和角度覆盖验证、候选融合。圆环寻找暗环包围的孔；实体顶帽寻找外轮廓。`jhb.geometry.*` 独立配置，默认 `method: contour`，也可改为 `hough`。找圆范围默认是顶帽框短边的0.35～0.55倍。继续使用 `top.refine_circle`、`top.allow_bbox_fallback`、`top.roi_padding_ratio`、`top.minimum_roi_size_px` 控制精定位及回退。找不到可信圆时默认使用顶帽框中心；关闭回退后找圆失败即无中心。定位和判色均成功时 `valid` 才为 true。

判色读取包内 `color_lab_v3` 的均值、逆协方差和 logdet。在原图÷2上取顶帽中心0.7倍半径内的核心像素，步长4，白平衡后取BGR中位数再转CIE Lab。白平衡根据排除物料后的明亮中性背景估计，缺少参考时给出警告并使用单位增益；模型包未提供其原车的白平衡代码，本实现为按说明补充的估计。`jhb.white_balance: false` 可关闭，便于对比。

颜色编号仍是红1、黄2、蓝3、绿4、黑5、浅蓝6；保留同色多实例。判色后验低于0.9或 d² 大于23.172时返回 `COLOR_REJECTED`、`color_id: 0`、`color_name: unknown`，保留已取得的 `center_px/center_source`，`valid` 仍为 false；该中心仅供原料区任意物料初始跟随，不能用于指定颜色追踪或抓取。距离门限按包内 README 的23.172配置（JSON中的 `chi2` 是16.27），可用 `jhb.color_d2_max` 修改。网页显示 Lab 后验、距离及拒判原因，保留诊断框/圆心。`target_color_id` 筛选已判定颜色，未知颜色保留用于诊断和初始跟随；任务 `geometry_target` 先粗判颜色，仅对所选颜色和未确定颜色精定位。

所有输出框、圆心、半径均还原到**传入原图**坐标，`search_roi` 的偏移也会还原；`jhb.work_scale: 4` 仅控制检测工作图，匹配包内2960×1664原图。输入较小图片或1280×720相机时可设为1或2并验证效果，已缩成工作图的图片用1。几何在原图ROI上执行，不受工作图或预览缩放影响。

默认 `jhb.model_path` 是带float32反量化输出的 `713700d9dc72.bin`，使用 `hobot_dnn.pyeasy_dnn`，输入为BT.601全范围NV12。当前适配器不支持 `ab57b8c392b6.bin` 的INT32输出，选择它会明确报错；不自动切换到ONNX或旧模型。ONNX用OpenCV DNN，不新增Python依赖。`detect_boxes` 仍只接受旧六色模型结果；JHB使用 `detect` 完成配对、几何和判色。

2026-11-16验证：包内自检图检出3个物料和3个顶帽，检测框中心满足包内1.5工作图像素容差；蓝、绿通过判色，浅蓝被拒判，三顶帽的几何结果均回退框中心。六类Lab均值色块、实体顶面轮廓、拒判、配对、多实例、ROI偏移、真实图片HTTP接口及构建后的ROS图片节点均通过检查。这些结果不代表六色实拍准确率或抓取精度。当前环境BPU驱动不可用，X5 `.bin` 及实时相机效果尚未实测。

## 网页结果

网页展示全部候选、颜色、模型标签、置信度、状态和中心来源：

- 青色十字 B：原检测框中心。
- 洋红色点/圆 C：真实几何圆心（原顶面模式为霍夫，JHB 默认轮廓/椭圆）；整体模式显示“不适用”，顶面未找到圆显示“未取得”。
- 黄色菱形 F：采用的定位中心，HSV 时为轮廓质心；JHB 颜色拒判时仅供初始跟随。

结果坐标是原始图像像素，原点左上、x 向右、y 向下；网页缩放不改变坐标。图像与结果一次成对返回。分别统计采集、识别处理、浏览器展示 FPS；静态图片不统计实时帧率，也不重复识别。

`BBOX_ONLY` 表示可用的框中心估计，不表示圆定位成功。六色 YOLO 框完全位于画面外、HSV 原框触边、JHB 顶帽边距不足、颜色歧义、断流、过期、推理异常时不给有效最终中心；网页断网时清空画面和坐标。原料停稳、选哪个物块、抓取顺序和准心由后续区域程序负责。整体框中心与顶面中心存在随位置变化的偏移，需要各自的对准准心。

## 模型与依赖

默认使用 `/root/material_deploy_v1` 下的两个用户模型：

| 路径 | 模型 |
|---|---|
| 顶面 | `export/upper_top/compiled_int8/upper_top_bayese_640x640_nv12.bin` |
| 整体 | `export/whole/compiled_int8/whole_bayese_640x640_nv12.bin` |

配置中 `top.classes_path`、`whole.classes_path` 对应实际六类标签文件，启动时核对顺序。`model_runtime_path` 指向部署目录的 `rdk_model_zoo`，依赖板端 X5 BPU 驱动及 `hobot_dnn`；输入640×640 NV12，由官方运行库完成 letterbox、DFL、一次类内 NMS 和坐标反变换。

每次只加载选中的一个模型，不下载权重，不修改部署资料，失败不自动切 HSV 或另一模型。报告 `MODEL_NOT_CONFIGURED`、`MODEL_LOAD_FAILED`、`MODEL_INFERENCE_FAILED`。HSV 和手动几何不需要 BPU。

圆环与物料部署包含同名 Python 运行库包。同进程已经导入另一部署的 `samples/utils` 时明确报错；独立工具各自在进程中启动。以后统一区域节点时需确认共用运行库版本，不能仅反复改变 `sys.path`。

## ROS 与图片输出

```bash
source /opt/ros/humble/setup.bash
cd /root/工创/ros2_ws
colcon build --symlink-install --packages-select agv_perception
source /root/工创/ros2_ws/install/setup.bash
ros2 run agv_perception material_debug_node --ros-args \
  --params-file /root/工创/ros2_ws/src/agv_perception/config/material.yaml \
  -p input_image:=/root/material_deploy_v1/export/upper_top/test_images/check.jpg \
  -p output_dir:=/tmp/material_debug_image
```

静态图完成后退出，输出 `result.json`、`annotated.jpg`，以及实际执行路径的 HSV 掩膜或顶面灰度、边缘、候选图。图片不打开相机。相机模式输出 `results.jsonl`；`save_every_n_frames` 非0时抽样写入 `frame_帧号/`，默认不写相机图片。网页同样支持 YAML 的 `output_dir`。

启动 ROS 相机调试：

```bash
ros2 launch agv_perception material_debug.launch.py \
  config:=/root/工创/ros2_ws/src/agv_perception/config/material.yaml
```

无需模型的手动顶面调试（颜色由人工指定）：

```bash
ros2 run agv_perception material_debug_node --ros-args \
  --params-file /root/工创/ros2_ws/src/agv_perception/config/material.yaml \
  -p debug_mode:=geometry_only \
  -p input_image:=/root/material_deploy_v1/export/upper_top/test_images/check.jpg \
  -p 'manual_roi:=[368,404,481,519]' -p manual_color_id:=1 \
  -p output_dir:=/tmp/material_debug_manual
```

手动身份为 `manual`，不冒充模型识别。`show_preview: true` 需要桌面环境，q/Esc退出；SSH 推荐网页入口。

## 核心接口

```python
from agv_perception.material_config import read_material_config
from agv_perception.material_detector import MaterialDetector
from agv_perception.material_model import load_material_model

config = read_material_config('/root/工创/ros2_ws/src/agv_perception/config/material.yaml')
detector = MaterialDetector(config, load_material_model(config))
result = detector.detect(frame_bgr, frame_seq=12, stamp=read_time_s)
for item in result.observations:
    if item.valid:
        print(item.color_id, item.center_px, item.center_source)
```

也可用 `parse_material_config(parameters)` 从参数映射构建配置。`detect(..., target_color_id=None, search_roi=None)` 默认使用 YAML；`target_color_id=0` 强制返回全部颜色，`search_roi=[]` 强制全图。`search_roi` 为原图 `[x1,y1,x2,y2]` 整数区域。工作 ROI 和顶面 ROI 偏移均还原到原图。

`detect_boxes(frame, detections, frame_seq=..., stamp=...)` 接收 `MaterialModelDetection(color_id, raw_label, confidence, bbox_xyxy)` 列表，框必须已经是所传整图坐标，不再执行 NMS/letterbox。`detect_roi(frame, bbox_xyxy, manual_color_id, ...)` 为显式手动顶面入口。

结果中 B/C/F 分开保存：`bbox_center_px`、`circle_center_px`、`center_px`；`center_source` 为 `circle/bbox/contour_centroid`。`valid=false` 时最终 `center_px` 与来源为空，诊断框中心可保留。`result.to_dict()` 可直接 JSON 序列化，不含图像数组；本地 `debug` 保留掩膜、边缘和候选。HSV 清空某色阈值时帧状态为 `CONFIGURATION_INCOMPLETE`，`warnings` 列出缺失颜色。

## 检查与当前验证范围

```bash
cd /root/工创
OPENBLAS_NUM_THREADS=1 PYTHONPATH=ros2_ws/src/agv_perception python3 -m unittest discover -s ros2_ws/src/agv_perception/test -v
OPENBLAS_NUM_THREADS=1 PYTHONPATH=tools/material_debug:ros2_ws/src/agv_perception python3 -m unittest discover -s tools/material_debug/test -v
node tools/material_debug/test/test_web.js
AGV_TEST_X5_MATERIAL=1 OPENBLAS_NUM_THREADS=1 PYTHONPATH=tools/material_debug:ros2_ws/src/agv_perception python3 -m unittest discover -s tools/material_debug/test -p test_x5.py -v
```

真实模型测试需要 X5 设备访问权限。2026-11-09 已验证两个模型的两张原始图片，覆盖六色；顶面六个样本中5个圆心成功、黄色样本框中心兜底。这是有限样本验证，不是六色准确率或抓取成功率。当前执行环境未发现相机视频设备，实时采集、实际 FPS、各区域准心及爪子容差仍待接入相机后验证。详细记录见[验证记录](/root/工创/docs/相机检查记录/物料识别工具验证记录.md)。

同日连续计时已确认BPU实际执行：预热后核心顶面＋霍夫平均36.83ms、27.14FPS，整体平均29.51ms、33.88FPS。当前预览调度会降低输出率；30Hz原图回放＋独立HTTP客户端下，新结果约8.79/10.90FPS，不能把核心FPS当成网页FPS。测量方法、P95及BPU驱动证据见验证记录末尾性能实测部分。

2026-11-17追加验证：网页选择控件、HTTP模式接口、真实ONNX JHB与HSV三次往返切换、同一静态图片重新处理、加载失败保留原模式及推理中切换丢弃旧结果均通过测试。物料网页Python检查9项通过、1项X5测试因缺少板端条件跳过，网页JavaScript检查通过。
