"""Single-axis relative motion measured in the initial body frame."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class RelativeMove:
    axis: str
    target: float
    speed: float
    tolerance: float

    def __post_init__(self):
        if self.axis not in ('x', 'y', 'yaw'):
            raise ValueError('axis must be x, y or yaw')
        if not all(math.isfinite(v) for v in (self.target, self.speed, self.tolerance)):
            raise ValueError('move parameters must be finite')
        if self.speed <= 0 or self.tolerance <= 0 or abs(self.target) <= self.tolerance:
            raise ValueError('speed/tolerance must be positive; |target| must exceed tolerance')

    def displacement(self, origin, current):
        """Poses are (x metres, y metres, continuous yaw radians)."""
        dx, dy = current[0]-origin[0], current[1]-origin[1]
        c, s = math.cos(origin[2]), math.sin(origin[2])
        if self.axis == 'x':
            return c*dx+s*dy
        if self.axis == 'y':
            return -s*dx+c*dy
        return math.degrees(current[2]-origin[2])

    def remaining(self, displacement):
        return abs(self.target)-math.copysign(1., self.target)*displacement

    def command(self, displacement):
        remaining = self.remaining(displacement)
        if remaining <= self.tolerance:
            return (0., 0., 0.)
        slowdown = 15. if self.axis == 'yaw' else .05
        minimum = min(self.speed, 2. if self.axis == 'yaw' else .01)
        speed = min(self.speed, max(minimum, self.speed*remaining/slowdown))
        values = [0., 0., 0.]
        values[{'x': 0, 'y': 1, 'yaw': 2}[self.axis]] = math.copysign(speed, self.target)
        return tuple(values)
