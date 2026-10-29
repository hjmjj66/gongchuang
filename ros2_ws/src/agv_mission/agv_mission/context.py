from dataclasses import dataclass, field
import time
import uuid

from .stages import Stage


@dataclass
class ActiveTask:
    task_id: str
    kind: str
    executor: str
    accepted: bool = False
    handle: object = None
    terminal: object = None
    started_at: float = field(default_factory=time.monotonic)
    spec: tuple = ()
    goal: int = 0
    batch: int = 0
    terminal_seen_at: float = 0.0


@dataclass
class MissionContext:
    home_node: int
    run_id: str = field(default_factory=lambda: str(uuid.uuid4()))
    state: str = 'INITIALIZING'
    stage: Stage = Stage.STARTUP_SCAN
    substate: str = ''
    active: object = None
    code: object = None
    pending_code: object = None
    first_layer: dict = field(default_factory=dict)
    confirmed_steps: dict = field(default_factory=dict)
    confirmed_node: int = 0
    position_known: bool = False
    current_heading: str = ''
    map_revision: int = 0
    blocked_edges: tuple = ()
    manual_forbidden_routes: tuple = ()
    manual_blocked_edges: tuple = ()
    reason: str = ''
    quiescent: bool = False
    arm_pending: bool = False
    started_at: float = 0.0
    finished_at: float = 0.0
    stage_started_at: float = 0.0

    def matches(self, run_id, task_id):
        return bool(self.active and run_id == self.run_id and task_id == self.active.task_id)
