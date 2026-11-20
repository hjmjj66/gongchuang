"""Shared observations, PID and fixed-frame to body-frame return control."""

from dataclasses import dataclass, field
import math


def clamp(value, limit):
    return max(-limit, min(limit, value))


def target_uv(config, image_size, *, stack=False):
    vision = config['vision']
    point = vision.get('stack_target_uv_px', vision['target_uv_px']) if stack else vision['target_uv_px']
    return point or [(n-1)/2 for n in image_size]


class PID:
    def __init__(self, gains, limit):
        self.kp, self.ki, self.kd = (gains[k] for k in ('kp', 'ki', 'kd'))
        self.limit = limit
        self.reset()

    def reset(self):
        self.integral = 0.
        self.previous = self.at = None

    def update(self, error, now):
        dt = 0. if self.at is None else now-self.at
        derivative = 0. if dt <= 0 else (error-self.previous)/dt
        candidate = self.integral + error*max(0., dt)
        raw = self.kp*error+self.ki*candidate+self.kd*derivative
        if abs(raw) <= self.limit or raw*error < 0:
            self.integral = candidate
        self.previous, self.at = error, now
        return clamp(raw, self.limit)


def line_slope(centers, min_span):
    if len(centers) < 2 or max(u for u, _ in centers)-min(u for u, _ in centers) < min_span:
        return None
    mu, mv = (sum(p[i] for p in centers)/len(centers) for i in (0, 1))
    return sum((u-mu)*(v-mv) for u, v in centers)/sum((u-mu)**2 for u, _ in centers)


@dataclass(frozen=True)
class Observation:
    kind: str
    sequence: int
    captured_at: float
    image_size: tuple
    rings: dict
    materials: tuple = ()  # (color_id, center_px); color 0 denotes an unknown color.
    center_sources: dict = field(default_factory=dict)
    geometry_target: int = 0  # 0: all targets refined; otherwise the ring/color refined in this frame.


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float
    received_at: float
    vx: float = 0.
    vy: float = 0.


@dataclass(frozen=True)
class GantryPose:
    fb_pulses: int
    yaw_rad: float
    received_at: float

    @classmethod
    def from_status(cls, status, received_at):
        # A zero reading with bit0 clear is unknown, not a measured home pose.
        if status.pos_flags & 0x01:
            return cls(status.fb_pulses, status.yaw_rad, received_at)
        return None


def return_velocity(pids, pose, target, config):
    x = pids[0].update(target[0]-pose.x, pose.received_at)
    y = pids[1].update(target[1]-pose.y, pose.received_at)
    cs, sn = math.cos(pose.yaw), math.sin(pose.yaw)
    return (clamp(cs*x+sn*y, config['max_vx_mps']),
            clamp(-sn*x+cs*y, config['max_vy_mps']), 0.)
