"""Fixed-frame geometry for x FORWARD, y LEFT, CCW-positive yaw."""
import math


def body_to_fixed(x, y, yaw_rad):
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    return c*x-s*y, s*x+c*y


def projection(start, current, direction):
    dx, dy = current[0]-start[0], current[1]-start[1]
    ex, ey = direction
    return 1000.*(dx*ex+dy*ey), 1000.*(ex*dy-ey*dx)
