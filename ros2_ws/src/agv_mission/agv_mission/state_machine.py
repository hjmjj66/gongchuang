"""State transitions with no ROS dependencies or implicit I/O."""

import time

from .stages import Stage


def start(context):
    if context.state != 'READY':
        raise ValueError('仅READY允许开始；暂停或故障后必须复位')
    context.state = 'RUNNING'
    context.quiescent = False
    context.started_at = time.monotonic()
    context.stage = Stage.EXIT_START_AREA


def pause(context):
    if context.state in ('PAUSING', 'PAUSED'):
        return
    if context.state != 'RUNNING':
        raise ValueError('仅运行中的比赛允许暂停')
    context.state = 'PAUSING'       # Close the transition gate before sending stop requests.
    context.reason = '用户暂停；必须复位后重新开始'
    context.quiescent = False


def advance(context):
    if context.state != 'RUNNING':
        return False
    context.active = None
    if context.stage == Stage.ENTER_FINISH_AREA:
        context.state = 'FINISHED'
        context.quiescent = True
        context.finished_at = time.monotonic()
    else:
        context.stage = (Stage.STARTUP_SCAN if context.stage == Stage.EXIT_START_AREA else
                         Stage.ENTER_FINISH_AREA if context.stage == Stage.RETURN_HOME else
                         Stage(context.stage + 1))
        context.substate = 'ENTER'
    return True


def commit_steps(context, task, records):
    """Validate full records before mutating the ledger; retransmissions are idempotent."""
    expected = {row[0]: row[1:] for row in task.spec}
    if [record.step_id for record in records] != [row[0] for row in task.spec[:len(records)]]:
        raise ValueError('步骤确认必须是按执行顺序排列的累计前缀')
    pending = {}
    for record in records:
        if record.run_id != context.run_id or record.task_id != task.task_id:
            raise ValueError('步骤身份不匹配')
        if record.batch != task.batch:
            raise ValueError('步骤批次不匹配')
        value = (record.operation, record.item_index, record.color, record.position)
        if expected.get(record.step_id) != value or not record.controller_completed:
            raise ValueError('步骤内容不符合当前任务或机构尚未完成')
        key = (record.run_id, record.task_id, record.step_id)
        if key in context.confirmed_steps and context.confirmed_steps[key] != value:
            raise ValueError('同一步骤的重复结果冲突')
        if key in pending and pending[key] != value:
            raise ValueError('结果包含冲突步骤')
        pending[key] = value
    new = {key: value for key, value in pending.items() if key not in context.confirmed_steps}
    context.confirmed_steps.update(pending)
    return new
