"""Route segments and position/angle feedback control."""

from dataclasses import dataclass
from math import sqrt
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


class AnglePID:
    """Degrees in, degrees/second out; derivative uses measured angular rate."""

    def __init__(self, kp, ki, kd, max_speed, tolerance):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.max_speed, self.tolerance = max_speed, tolerance
        self.reset()

    def reset(self):
        self.integral = 0.0

    def update(self, error, angular_rate, dt):
        if abs(error) <= self.tolerance:
            self.reset()
            return 0.0
        integral = self.integral + error * dt if self.ki else 0.0
        output = self.kp * error + self.ki * integral - self.kd * angular_rate
        # Freeze integration only when it would push farther into saturation.
        if abs(output) <= self.max_speed or output * error < 0:
            self.integral = integral
        output = self.kp * error + self.ki * self.integral - self.kd * angular_rate
        return max(-self.max_speed, min(self.max_speed, output))


class PositionPID:
    """Metres in, metres/second out, with independent XY speed limits."""

    def __init__(self, kp, ki, kd, max_speeds, tolerance):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.max_speeds, self.tolerance = max_speeds, tolerance
        self.reset()

    def reset(self):
        self.integral = [0.0, 0.0]

    def update(self, errors, velocities, dt):
        output = []
        for axis, (error, velocity, limit) in enumerate(zip(errors, velocities, self.max_speeds)):
            if abs(error) <= self.tolerance:
                self.integral[axis] = 0.0
                output.append(0.0)
            else:
                integral = self.integral[axis] + error * dt if self.ki else 0.0
                value = self.kp * error + self.ki * integral - self.kd * velocity
                if abs(value) <= limit or value * error < 0:
                    self.integral[axis] = integral
                value = self.kp * error + self.ki * self.integral[axis] - self.kd * velocity
                output.append(max(-limit, min(limit, value)))
        return tuple(output)


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

