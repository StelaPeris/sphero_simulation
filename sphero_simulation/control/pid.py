import time
from .util import clamp, wrap180


def _dt(obj, now, default):
    now = time.time() if now is None else now
    dt = (now - obj.prev_t) if obj.prev_t else default
    obj.prev_t = now
    return dt if dt > 0 else default


# ==================== YAW PID (vanjska petlja) ====================
class YawPID:
    def __init__(self, kp=0.22, ki=0.0, kd=0.18, roll_limit=15.0, invert=False, deadzone=2.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.roll_limit = roll_limit
        self.invert = invert           # True -> out = -out (iz configa, yaw_invert)
        self.deadzone = deadzone       # deg, mrtva zona greske
        self.reset()

    wrap180 = staticmethod(wrap180)

    def reset(self):
        self.integral = 0.0
        self.prev_measured = None 
        self.prev_t = None
        self.d_meas_filtered = 0.0

    def update(self, target_yaw, meas_yaw, now=None):
        dt = _dt(self, now, 0.02)

        err = wrap180(target_yaw - meas_yaw)
        if abs(err) < self.deadzone:   # mrtva zona
            err = 0.0

        if self.ki > 0:
            self.integral += err * dt
            self.integral = clamp(self.integral, 10.0)
        else:
            self.integral = 0.0

        if self.prev_measured is None:
            self.prev_measured = meas_yaw

        # D po mjerenju + LPF
        d_meas_raw = wrap180(meas_yaw - self.prev_measured) / dt
        self.prev_measured = meas_yaw
        self.d_meas_filtered = 0.2 * d_meas_raw + 0.8 * self.d_meas_filtered

        out = self.kp * err + self.ki * self.integral - self.kd * self.d_meas_filtered
        if self.invert:        
            out = -out

        return clamp(out, self.roll_limit)


# ==================== ROLL PID (unutarnja petlja) ====================
class RollPID:
    def __init__(self, kp=0.02, ki=0.001, kd=0.003, alpha_d=0.2):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.alpha_d = alpha_d
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_measured = None
        self.prev_t = None
        self.d_filtered = 0.0

    def update(self, target, measured, now=None):
        dt = _dt(self, now, 0.02)

        err = target - measured

        # Anti-windup s mrtvom zonom
        if abs(err) < 5.0:
            self.integral += err * dt
            self.integral = clamp(self.integral, 10.0)
        else:
            self.integral = 0.0

        if self.prev_measured is None:
            self.prev_measured = measured

        d_raw = (measured - self.prev_measured) / dt
        self.prev_measured = measured
        self.d_filtered = self.alpha_d * d_raw + (1.0 - self.alpha_d) * self.d_filtered

        out = self.kp * err + self.ki * self.integral - self.kd * self.d_filtered
        return clamp(out, 1.0)


# ==================== SPEED PID ====================
class SpeedPID:
    def __init__(self, kp, ki, kd, out_lim=1.0, out_min=0.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_lim = out_lim
        self.out_min = out_min
        self.reset()

    def update(self, target, measured, now=None):
        dt = _dt(self, now, 0.05)
        err = target - measured

        if self.prev_measured is None:
            self.prev_measured = measured
        d_measured = (measured - self.prev_measured) / dt
        self.prev_measured = measured

        unsat = self.kp*err + self.ki*self.integral - self.kd*d_measured
        out = max(self.out_min, min(self.out_lim, unsat))

        if out == unsat or (unsat > self.out_lim and err < 0) or (unsat < self.out_min and err > 0):
            self.integral += err * dt
        return out

    def reset(self):
        self.integral = 0.0
        self.prev_measured = None
        self.prev_t = None