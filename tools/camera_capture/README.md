# 主摄像头网页拍照工具

在 X5 上运行服务，电脑通过浏览器实时看主摄像头，按空格或点击“拍照保存”，将完整原图保存到板子。无需模型、ROS2 节点、colcon 构建、Node.js、pip Web 框架或外网资源。

## 启动

依赖板子已有的 Python 3、OpenCV 和 NumPy。复用源码中的 `agv_perception.camera.CameraStream`，只打开主摄像头，不启动识别或运动节点。先关闭其他使用这只相机的程序。

在板子项目根目录运行：

```bash
./tools/camera_capture/start.sh
```

电脑与板子在同一网络，在电脑浏览器访问 `http://板子IP:8082`。板子本机可访问 `http://127.0.0.1:8082`。可用 `hostname -I` 查看板子 IP。

当前板子的连接地址（2026-10-31 记录）：

| 连接方式 | 板子 IP（SSH 地址） | 拍照网页 |
| --- | --- | --- |
| 电脑通过网线连接板子 | `192.168.127.10` | [http://192.168.127.10:8082](http://192.168.127.10:8082) |
| 板子连接手机热点，电脑也连接该热点 | `10.157.133.165` | [http://10.157.133.165:8082](http://10.157.133.165:8082) |

按当前连接方式选择地址，网页端口均为默认的 `8082`。手机热点分配的 IP 可能在重新连接后变化；若无法访问，在板子的 SSH 终端执行 `hostname -I` 核对当前地址。

默认设备沿用历史主相机检查记录，现场先核对 `ls -l /dev/v4l/by-id/`。默认请求 1280×720、MJPG、30 FPS、4 个缓冲；网页显示相机实际输出分辨率。

指定保存目录或设备：

```bash
./tools/camera_capture/start.sh --output-dir /home/工创/wuliao/datasets/rings
./tools/camera_capture/start.sh --device /dev/video0 --port 8083
```

所有选项：

```bash
./tools/camera_capture/start.sh --help
```

## 使用与文件

1. 等待相机画面稳定、按钮启用，检查视角及分辨率。
2. 点击拍照按钮，或让网页保持前台并按一次空格。长按不会连续拍照；保存中按钮暂时禁用。
3. 网页显示成功张数、最近保存的照片、文件名和板端目录。失败不会增加成功计数。
4. 在不同位置、角度和光照下重复拍摄；在板端终端按 Ctrl+C 退出。

默认保存到板子上本项目的 `工创/ring_dataset_raw/`（当前项目路径为 `/root/工创/ring_dataset_raw/`），不是浏览器电脑的下载目录。目录根据工具所在的项目位置确定，不受启动时所在目录影响。每次启动新建批次目录，不覆盖之前的照片；此目录已加入 Git 忽略规则。例如：

```text
/root/工创/ring_dataset_raw/
└── 20261031_143015_123456/
    ├── ring_20261031_143020_234567_000001.jpg
    └── ring_20261031_143025_345678_000002.jpg
```

文件名不包含目标编号；一张图可以同时包含 1、2、3，编号在后续标注阶段填写。可通过不同前缀区分采集场景：

```bash
./tools/camera_capture/start.sh --prefix rough
./tools/camera_capture/start.sh --prefix temporary
```

保存的是相机完整分辨率的 JPEG，质量 95，无界面文字、检测框、缩放或裁剪。网页预览默认限制为宽 960、约 10 FPS、JPEG 质量 75，以降低网络开销，**不影响保存图片**。可通过 `--preview-width`、`--preview-fps` 调整预览。

拍照取板端收到请求时的最新相机帧。网络预览存在延迟，精确构图时先停稳小车、等画面稳定后拍照。相机断开、画面超过 1 秒未更新或刚恢复尚未稳定时，后端拒绝保存，不会把旧画面冒充新照片。请求超时表示结果尚未确认，请查看照片计数或目录，不会自动重发拍照请求。

复制到电脑（在电脑终端执行，替换用户名和 IP）：

```bash
scp -r 用户名@板子IP:/root/工创/ring_dataset_raw ./
```

如果项目放在其他位置，或自定义了保存目录，复制命令也使用相应目录；以网页显示的板端保存路径为准。以前保存在 `~/ring_dataset_raw/` 的照片不会自动移动，重启工具后新照片使用上述默认目录。

## 验证

从项目根目录执行。Python 测试使用真实图像编码、文件写入及本机 HTTP，不连接真实摄像头；Node.js 仅用于前端交互测试，板端运行不需要安装。

```bash
PYTHONPATH=tools/camera_capture:ros2_ws/src/agv_perception python3 -m unittest discover -s tools/camera_capture/test -p 'test_*.py' -v
node tools/camera_capture/test/test_web.js
```

上板验收：打开页面，分别点击按钮和按空格各拍一张；确认长按空格只增加一张；检查板端 JPEG 的实际分辨率及内容；断开相机后确认不能继续保存。当前开发环境没有连接 X5，相机采集、浏览器真实渲染和现场网络性能仍需上板验证。
