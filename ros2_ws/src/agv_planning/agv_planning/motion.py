"""Small helpers for merging route actions and generating deceleration profiles."""

from dataclasses import dataclass
from math import hypot, isfinite, sqrt
from typing import Iterable

from .planner import Opcode, Step


_DIAGONAL_UNIT = sqrt(0.5)
TRANSLATIONS = {
    Opcode.FORWARD_ONE_EDGE: (1.0, 0.0),
    Opcode.BACKWARD_ONE_EDGE: (-1.0, 0.0),
    Opcode.STRAFE_LEFT_ONE_EDGE: (0.0, 1.0),
    Opcode.STRAFE_RIGHT_ONE_EDGE: (0.0, -1.0),
    Opcode.DIAGONAL_FORWARD_LEFT: (_DIAGONAL_UNIT, _DIAGONAL_UNIT),
    Opcode.DIAGONAL_FORWARD_RIGHT: (_DIAGONAL_UNIT, -_DIAGONAL_UNIT),
    Opcode.DIAGONAL_BACKWARD_LEFT: (-_DIAGONAL_UNIT, _DIAGONAL_UNIT),
    Opcode.DIAGONAL_BACKWARD_RIGHT: (-_DIAGONAL_UNIT, -_DIAGONAL_UNIT),
}


@dataclass(frozen=True)
class MotionSegment:
    opcode: Opcode
    amount: float
    end_node: int
    target_heading: str


def merge_steps(steps: Iterable[Step]):
    segments = []
    for step in steps:
        opcode = Opcode(step.opcode)
        amount = float(step.distance_mm) if opcode in TRANSLATIONS else 90.0
        if opcode in TRANSLATIONS and all(TRANSLATIONS[opcode]):
            amount *= sqrt(2.0)
        if segments and segments[-1].opcode == opcode:
            previous = segments[-1]
            segments[-1] = MotionSegment(
                opcode, previous.amount + amount, step.to_node, step.target_heading
            )
        else:
            segments.append(MotionSegment(opcode, amount, step.to_node, step.target_heading))
    return tuple(segments)


def ramp_speed(remaining: float, deceleration_distance: float,
               fast_speed: float, slow_speed: float, tolerance: float = 0.0):
    values = (remaining, deceleration_distance, fast_speed, slow_speed, tolerance)
    if not all(isfinite(value) for value in values):
        raise ValueError('speed profile values must be finite')
    if tolerance < 0 or deceleration_distance <= tolerance:
        raise ValueError('deceleration distance must exceed a nonnegative tolerance')
    if fast_speed <= 0 or slow_speed <= 0 or slow_speed > fast_speed:
        raise ValueError('speeds must be positive and slow speed must not exceed fast speed')
    if remaining <= tolerance:
        return 0.0
    if deceleration_distance <= 0 or remaining >= deceleration_distance:
        return fast_speed
    ratio = min(1.0, max(0.0,
                         (remaining - tolerance) / (deceleration_distance - tolerance)))
    return hypot(slow_speed * sqrt(1.0 - ratio), fast_speed * sqrt(ratio))

