"""Temporary copies keep parameter-save tests away from the vehicle's settings."""
from pathlib import Path

from agv_perception.work_config import read_work_config

ROOT = Path(__file__).resolve().parents[3]


def copy_debug_config(path):
    path = Path(path)
    source = ROOT/'tools/coarse_debug/config/debug.yaml'
    motion = ROOT/'ros2_ws/src/agv_perception/config/work_motion.yaml'
    target = path.parent/'work_motion.yaml'
    target.write_text(motion.read_text())
    path.write_text(source.read_text().replace(str(motion), str(target)))
    return read_work_config(path)
