import math


def clamp(val, limit):
    return max(-limit, min(limit, val))


def wrap180(deg):
    return math.degrees(math.atan2(math.sin(math.radians(deg)),
                                   math.cos(math.radians(deg))))


def heading_from_quat(w, x, y, z):
    fx = 1.0 - 2.0 * (y*y + z*z)
    fy = 2.0 * (x*y + w*z)
    return math.degrees(math.atan2(fy, fx))