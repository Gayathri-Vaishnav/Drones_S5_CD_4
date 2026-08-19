"""pid_controller.py — PID velocity controller with robust landing transition."""

import numpy as np

KP = 0.20
KI = 0.03
KD = 0.35


class PIDAxis:
    def __init__(self, kp=KP, ki=KI, kd=KD, integral_limit=2.0, output_limit=3.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.output_limit   = output_limit
        self.reset()

    def reset(self):
        self._integral = 0.0
        self._last_err = 0.0
        self._first = True

    def update(self, err: float) -> float:
        if self._first:
            self._last_err = err
            self._first = False

        self._integral += err
        self._integral = float(np.clip(self._integral, -self.integral_limit, self.integral_limit))

        d_err = err - self._last_err
        v_cmd = self.kp * err + self.ki * self._integral + self.kd * d_err
        self._last_err = err
        return float(np.clip(v_cmd, -self.output_limit, self.output_limit))


class ProportionalAxis:
    def __init__(self, kp=KP, output_limit=3.0):
        self.kp = kp
        self.output_limit = output_limit

    def update(self, err: float) -> float:
        return float(np.clip(self.kp * err, -self.output_limit, self.output_limit))


class DroneController:
    """Full 3-axis PID controller for the quadrotor."""

    DESCENT_SPEED_HIGH = 0.85   # m/s (point B -> C)
    DESCENT_SPEED_LOW  = 0.45   # m/s (point C -> D)
    FINAL_DESCENT      = 0.25   # m/s (point D -> touchdown)

    ALT_SWITCH   = 1.8          # m (switch to small tag)
    TOUCH_ALT    = 0.45         # m (final landing touchdown phase)
    LAND_ALT     = 0.15         # m (declared landed)
    HOVER_RADIUS = 0.45         # m

    MASS = 3.5
    GRAVITY = 9.81
    HOVER_THRUST = MASS * GRAVITY

    MAX_ACC = 4.0
    KP_V_LAT = 2.0
    KP_V_Z   = 2.2

    def __init__(self, use_pid=True):
        self.use_pid = use_pid
        if use_pid:
            self.pid_x = PIDAxis(kp=KP, ki=KI, kd=KD)
            self.pid_y = PIDAxis(kp=KP, ki=KI, kd=KD)
            self.pid_z = PIDAxis(kp=0.25, ki=0.02, kd=0.30)
        else:
            self.pid_x = ProportionalAxis(kp=KP)
            self.pid_y = ProportionalAxis(kp=KP)
            self.pid_z = ProportionalAxis(kp=0.25)

        self.phase = "SEARCH"
        self._vz_setpoint = 0.0
        self._last_known_alt = 12.0

    def reset(self):
        if self.use_pid:
            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_z.reset()
        self.phase = "SEARCH"
        self._vz_setpoint = 0.0
        self._last_known_alt = 12.0

    def update(self, lock, alt_meas, east_err, north_err, vel_ned, t):
        """vel_ned: [vx_north, vy_east, vz_down] where vz_down > 0 is descending."""
        vx, vy, vz = vel_ned
        if lock:
            self._last_known_alt = alt_meas
        alt = alt_meas if lock else self._last_known_alt
        r_lat = np.hypot(east_err, north_err) if lock else 999.0

        # Phase transitions
        if self.phase == "SEARCH" and lock:
            self.phase = "APPROACH"
        if self.phase == "APPROACH" and lock and r_lat < self.HOVER_RADIUS:
            self.phase = "DESCEND_HIGH"
        if self.phase == "DESCEND_HIGH" and alt < self.ALT_SWITCH:
            self.phase = "DESCEND_LOW"
        if (self.phase in ("DESCEND_HIGH", "DESCEND_LOW")) and alt < self.TOUCH_ALT:
            self.phase = "LAND"

        # Desired vertical velocity
        if self.phase in ("SEARCH", "APPROACH"):
            vz_cmd = 0.0
        elif self.phase == "DESCEND_HIGH":
            vz_cmd = self.DESCENT_SPEED_HIGH
        elif self.phase == "DESCEND_LOW":
            vz_cmd = self.DESCENT_SPEED_LOW
        else:  # LAND
            vz_cmd = self.FINAL_DESCENT

        self._vz_setpoint = vz_cmd

        # Lateral velocities
        if lock:
            vx_cmd = self.pid_x.update(north_err)
            vy_cmd = self.pid_y.update(east_err)
        else:
            vx_cmd = vy_cmd = 0.0

        ax = self.KP_V_LAT * (vx_cmd - vx)
        ay = self.KP_V_LAT * (vy_cmd - vy)

        a_lat = np.hypot(ax, ay)
        if a_lat > self.MAX_ACC:
            ax *= self.MAX_ACC / a_lat
            ay *= self.MAX_ACC / a_lat

        fx = ax * self.MASS
        fy = ay * self.MASS

        # Vertical force
        az = self.KP_V_Z * (vz_cmd - vz)
        az = float(np.clip(az, -3.5, 6.0))

        if self.phase == "LAND":
            base_thrust = self.HOVER_THRUST * 0.75
        else:
            base_thrust = self.HOVER_THRUST

        fz_up = float(np.clip(base_thrust - self.MASS * az, 0.0, 75.0))

        return fx, fy, fz_up, vz_cmd

    def is_landed(self, alt, vel_ned):
        return alt < self.LAND_ALT and abs(vel_ned[2]) < 0.35

    @property
    def current_phase(self):
        return self.phase
