import math
import numpy as np


class KalmanCV:
    """Stanje [x, y, vx, vy]. Predict po modelu, update na mjerenje pozicije.
    q: gustoca suma ubrzanja [m^2/s^3] (neovisno za x i y)
    r: varijanca suma pozicije s kamere [m^2] (1e-4 -> sigma = 1 cm)"""
    def __init__(self, q=1.0, r=1e-4):
        self.q = q
        self.r = r
        self.reset()

    def predict(self, dt):
        F = np.array([[1, 0, dt, 0],
                      [0, 1, 0, dt],
                      [0, 0, 1,  0],
                      [0, 0, 0,  1]], dtype=float)
        self.x = F @ self.x
        # neovisan sum ubrzanja za x i y
        q1 = self.q * np.array([[dt**4 / 4, dt**3 / 2],
                                [dt**3 / 2, dt**2]])
        Q = np.zeros((4, 4))
        Q[np.ix_([0, 2], [0, 2])] = q1     # x, vx
        Q[np.ix_([1, 3], [1, 3])] = q1     # y, vy
        self.P = F @ self.P @ F.T + Q

    def update(self, zx, zy):
        if not self.initialized:
            self.x[:] = [zx, zy, 0.0, 0.0]
            self.initialized = True
            return
        H = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0]], dtype=float)
        z = np.array([zx, zy])
        R = np.eye(2) * self.r
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P

    def speed(self):
        return math.hypot(self.x[2], self.x[3])

    def reset(self):
        self.x = np.zeros(4)
        self.P = np.eye(4) * 1.0
        self.initialized = False


class SpeedEstimator:
    def __init__(self, q=1.0, r=1e-4, pose_timeout=0.5, dir_tau=4.0, alpha_speed=0.5):
        self.kf = KalmanCV(q=q, r=r)
        self.pose_timeout = pose_timeout
        self.dir_tau = dir_tau            # s, vremenska konstanta smjera (>> 2.5 s ljuljanja)
        self.alpha_speed = alpha_speed
        self.last_pose_t = None           # vrijeme dolaska zadnje poze (za pose_ok)
        self.last_pose_stamp = None       # vrijeme snimanja zadnje poze (za dt filtra)
        self.speed_meas = self.speed_meas_filtered = 0.0
        self.dir_lp = np.zeros(2)
        self.speed_fwd = self.speed_fwd_filtered = 0.0

    def update(self, x, y, stamp, now):
        if stamp <= 0.0:
            stamp = now
        if self.last_pose_stamp is not None:
            dt = stamp - self.last_pose_stamp
            if dt <= 0.0:
                return                      # stara ili duplicirana poza
            if dt > self.pose_timeout:      # kamera je izgubila kuglu -> kreni ispocetka
                self.kf.reset()
                self.speed_meas_filtered = 0.0
                self.speed_fwd_filtered = 0.0
            elif dt > 1e-3:
                self.kf.predict(dt)
        self.last_pose_stamp = stamp
        self.kf.update(x, y)
        self.speed_meas = self.kf.speed()
        self.speed_meas_filtered = (self.alpha_speed * self.speed_meas +
                                    (1.0 - self.alpha_speed) * self.speed_meas_filtered)
        v = self.kf.x[2:4]
        if self.last_pose_t is not None:
            a = min(1.0, max(0.0, (now - self.last_pose_t) / self.dir_tau))
            self.dir_lp = (1.0 - a) * self.dir_lp + a * v
        n = np.linalg.norm(self.dir_lp)
        self.speed_fwd = float(v @ self.dir_lp / n) if n > 0.03 else self.speed_meas
        self.speed_fwd_filtered = (self.alpha_speed * self.speed_fwd +
                                   (1.0 - self.alpha_speed) * self.speed_fwd_filtered)
        self.last_pose_t = now

    def pose_ok(self, now):
        return self.last_pose_t is not None and (now - self.last_pose_t) < self.pose_timeout