# 雷达执行端占位

目前不打开雷达、不提供服务或 Action，也不发布扫描成功或空障碍结果。
`agv_mission/config/mission.yaml` 的 `use_lidar: false` 表示显式跳过雷达，
调度器只提交人工禁行地图。`true` 时统一启动器会加载本包的
`launch/obstacles.launch.py`；当前入口明确报错，避免将占位当作真实扫描。

后续真实实现需提供 `/obstacles/get_runtime`、`/obstacles/stop`、
`/obstacles/reset_context` 以及 `/obstacles/scan`（`ScanRoads` Action），
遵循比赛轮次、进程身份、REAL 来源、调度心跳和停止确认契约。
扫描结果仅返回雷达禁行边，人工禁行边由 mission 合并。实现节点与进程退出联动后，
在现有 launch 中启动即可，无需向调度器添加模拟成功分支。
