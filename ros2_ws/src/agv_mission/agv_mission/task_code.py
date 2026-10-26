"""Validate the four groups as a single immutable competition task."""

from dataclasses import dataclass
import re


@dataclass(frozen=True)
class TaskCode:
    raw: str
    batch1_colors: tuple
    batch1_positions: tuple
    batch2_colors: tuple
    batch2_positions: tuple

    @property
    def first_layer(self):
        return dict(zip(self.batch1_colors, self.batch1_positions))


def parse_task_code(text):
    if not isinstance(text, str) or not re.fullmatch(r'[1-6]{3}\+[1-3]{3}\+[1-6]{3}\+[1-3]{3}', text):
        raise ValueError('任务码必须为四组三位数字，例如156+123+516+231')
    colors1, positions1, colors2, positions2 = [tuple(map(int, group)) for group in text.split('+')]
    if len(set(colors1)) != 3 or len(set(colors2)) != 3:
        raise ValueError('每批颜色必须互不重复')
    if set(colors1) != set(colors2):
        raise ValueError('两批颜色集合必须相同')
    if set(positions1) != {1, 2, 3} or set(positions2) != {1, 2, 3}:
        raise ValueError('位置组必须为1、2、3的排列')
    return TaskCode(text, colors1, positions1, colors2, positions2)
