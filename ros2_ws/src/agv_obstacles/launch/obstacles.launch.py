"""Reserved entry point: never advertise a fabricated REAL scan result."""


def generate_launch_description():
    raise RuntimeError(
        'agv_obstacles 目前仅为占位包，真实雷达执行端尚未实现。'
        '请在 mission.yaml 中设置 use_lidar: false；实现后再启用 true。')
