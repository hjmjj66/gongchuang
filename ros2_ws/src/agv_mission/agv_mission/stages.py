"""Fixed competition stages and the parameters owned by the scheduler."""

from enum import IntEnum


class Stage(IntEnum):
    STARTUP_SCAN = 0
    MOVE_TO_QR = 1
    READ_TASK_CODE = 2
    MOVE_TO_RAW_1 = 3
    RAW_BATCH_1 = 4
    MOVE_TO_COARSE_1 = 5
    COARSE_BATCH_1 = 6
    MOVE_TO_STORAGE_1 = 7
    STORAGE_PLACE_1 = 8
    MOVE_TO_RAW_2 = 9
    RAW_BATCH_2 = 10
    MOVE_TO_COARSE_2 = 11
    COARSE_BATCH_2 = 12
    MOVE_TO_STORAGE_2 = 13
    STORAGE_STACK_2 = 14
    RETURN_HOME = 15
    # Keep existing work-stage IDs stable for external executors.
    EXIT_START_AREA = 16
    ENTER_FINISH_AREA = 17


LABELS = (
    '开局雷达扫描', '启停区→二维码区', '扫码', '二维码区→原料区',
    '第一批原料取料', '原料区→粗加工区', '第一批粗加工三放三取',
    '粗加工区→暂存区', '第一批暂存平放', '暂存区→原料区',
    '第二批原料取料', '原料区→粗加工区', '第二批粗加工三放三取',
    '粗加工区→暂存区', '第二批同色码垛', '返回启停区',
    '斜移离开启停区', '斜移进入结束启停区',
)
ROUTES = {1: (None, 2), 3: (2, 4), 5: (4, 6), 7: (6, 8),
          9: (8, 4), 11: (4, 6), 13: (6, 8), 15: (8, None)}
WORK = {4: ('raw', 'RAW_PICK', 1), 6: ('coarse', 'COARSE_PROCESS', 1),
        8: ('storage', 'STORAGE_PLACE', 1), 10: ('raw', 'RAW_PICK', 2),
        12: ('coarse', 'COARSE_PROCESS', 2), 14: ('storage', 'STORAGE_STACK', 2)}


def batch_spec(stage, code, confirmed_first_layer):
    area, operation, batch = WORK[stage]
    colors = code.batch1_colors if batch == 1 else code.batch2_colors
    positions = code.batch1_positions if batch == 1 else code.batch2_positions
    if operation == 'RAW_PICK':
        positions = ()
    elif operation == 'STORAGE_STACK':
        if confirmed_first_layer != code.first_layer:
            raise ValueError('第一层确认记录不完整或与任务码冲突')
        positions = tuple(confirmed_first_layer[color] for color in colors)
    return area, operation, batch, colors, positions


def expected_steps(operation, colors, positions):
    if len(colors) != 3 or len(set(colors)) != 3 or any(c not in range(1, 7) for c in colors):
        raise ValueError('整批必须包含三个不同的合法颜色')
    if operation == 'RAW_PICK':
        if positions:
            raise ValueError('原料取料不接受位置数组')
        positions = (0, 0, 0)
        phases = ('PICK',)
    else:
        if sorted(positions) != [1, 2, 3]:
            raise ValueError('放置位置必须为1、2、3的排列')
        phases = {'COARSE_PROCESS': ('PLACE', 'PICK'),
                  'STORAGE_PLACE': ('PLACE',), 'STORAGE_STACK': ('STACK',)}.get(operation)
        if phases is None:
            raise ValueError('未知整批操作')
    return tuple((f'{phase}:{i}', phase, i, color, position)
                 for phase in phases
                 for i, (color, position) in enumerate(zip(colors, positions), 1))
