"""quad_env.py — High-Fidelity MuJoCo Quadrotor Physics Environment.

Enhancements:
  - Direct hardware offscreen OpenGL camera rendering via mujoco.Renderer
  - Ground effect aerodynamic lift cushioning (z < 0.6m)
  - Motor first-order time response lag (tau = 0.04s)
  - Stochastic Dryden / Ornstein-Uhlenbeck wind gust turbulence
  - Ground contact friction & damping
"""

import os
import numpy as np
import mujoco

_XML = os.path.join(os.path.dirname(__file__), "assets", "quadrotor.xml")

MASS = 3.5          # kg (DJI M100)
GRAVITY = 9.81      # m/s²
HOVER_THRUST = MASS * GRAVITY   # 34.335 N
ROTOR_RADIUS = 0.17 # m


def quat_to_euler(q):
    w, x, y, z = q
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return np.array([roll, pitch, yaw])


def attitude_from_accel(acc):
    g = GRAVITY
    ax, ay, _ = acc
    roll = -np.arcsin(np.clip(ay / g, -1, 1))
    pitch = np.arcsin(np.clip(ax / g, -1, 1))
    return np.array([roll, pitch, 0.0])


class QuadEnv:
    """Realistic MuJoCo quadrotor environment."""

    DT_SIM = 0.002
    DT_CTRL = 0.02
    STEPS_PER_CTRL = int(DT_CTRL / DT_SIM)

    def __init__(self, xml_path=_XML, enable_camera_render=True):
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data  = mujoco.MjData(self.model)

        self._drone_jnt_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "drone_free")
        self._qpos_adr = self.model.jnt_qposadr[self._drone_jnt_id]
        self._qvel_adr = self.model.jnt_dofadr[self._drone_jnt_id]

        self._act_thrust = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "thrust_z")
        self._act_fx     = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "force_x")
        self._act_fy     = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "force_y")
        self._act_tz     = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "torque_z")

        self._vehicle_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "ground_vehicle")
        self._veh_jnt_x  = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "vehicle_x")
        self._veh_jnt_y  = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "vehicle_y")

        # Motor lag state
        self._thrust_act = HOVER_THRUST
        self._fx_act = 0.0
        self._fy_act = 0.0

        # Wind state (Dryden turbulence)
        self._wind = np.zeros(3)

        # Offscreen Camera Renderer
        self.renderer = None
        if enable_camera_render:
            try:
                self.renderer = mujoco.Renderer(self.model, 480, 640)
            except Exception as e:
                self.renderer = None

        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

    def reset(self, pos_ned=(1.2, -0.8, -12.0)):
        mujoco.mj_resetData(self.model, self.data)
        x_mj = pos_ned[0]
        y_mj = pos_ned[1]
        z_mj = -pos_ned[2]

        qpos = self.data.qpos
        qpos[self._qpos_adr + 0] = x_mj
        qpos[self._qpos_adr + 1] = y_mj
        qpos[self._qpos_adr + 2] = z_mj
        qpos[self._qpos_adr + 3] = 1.0
        qpos[self._qpos_adr + 4] = 0.0
        qpos[self._qpos_adr + 5] = 0.0
        qpos[self._qpos_adr + 6] = 0.0
        self.data.qvel[:] = 0.0

        self._thrust_act = HOVER_THRUST
        self._fx_act = 0.0
        self._fy_act = 0.0
        self._wind[:] = 0.0

        mujoco.mj_forward(self.model, self.data)

    def step(self, fx_north, fy_east, fz_up, torque_z=0.0, apply_wind=False):
        """Apply controls with motor lag, ground effect, and wind gust turbulence."""
        # 1. First-order Motor Response Lag (tau = 0.04s)
        alpha_m = self.DT_CTRL / (0.04 + self.DT_CTRL)
        self._thrust_act += alpha_m * (fz_up - self._thrust_act)
        self._fx_act     += alpha_m * (fx_north - self._fx_act)
        self._fy_act     += alpha_m * (fy_east - self._fy_act)

        # 2. Ground Effect Aerodynamic Cushion (z < 0.6m)
        alt = self.get_altitude()
        eff_thrust = self._thrust_act
        if fz_up >= HOVER_THRUST * 0.95 and 0.05 < alt < 0.60:
            ratio = (ROTOR_RADIUS / (4.0 * max(0.08, alt))) ** 2
            ge_factor = min(1.20, 1.0 / (1.0 - min(0.30, ratio)))
            eff_thrust *= ge_factor

        # 3. Wind Gust Turbulence
        if apply_wind:
            wind_noise = np.random.normal(0, [0.15, 0.15, 0.05], 3)
            self._wind = 0.96 * self._wind + wind_noise
            self._fx_act += self._wind[0] * MASS
            self._fy_act += self._wind[1] * MASS

        self.data.ctrl[self._act_thrust] = float(eff_thrust)
        self.data.ctrl[self._act_fx]     = float(self._fx_act)
        self.data.ctrl[self._act_fy]     = float(self._fy_act)
        self.data.ctrl[self._act_tz]     = float(torque_z)

        for _ in range(self.STEPS_PER_CTRL):
            mujoco.mj_step(self.model, self.data)

        # Safety contact floor
        if self.data.qpos[self._qpos_adr + 2] < 0.03:
            self.data.qpos[self._qpos_adr + 2] = 0.03
            if self.data.qvel[self._qvel_adr + 2] < 0:
                self.data.qvel[self._qvel_adr + 2] = 0.0

    def get_camera_frame(self):
        """Render onboard camera image directly through MuJoCo OpenGL offscreen renderer."""
        if self.renderer is not None:
            self.renderer.update_scene(self.data, camera="onboard_cam")
            return self.renderer.render()
        return None

    def get_state(self):
        qpos = self.data.qpos
        qvel = self.data.qvel

        x_mj = qpos[self._qpos_adr + 0]
        y_mj = qpos[self._qpos_adr + 1]
        z_mj = qpos[self._qpos_adr + 2]
        qw   = qpos[self._qpos_adr + 3]
        qx   = qpos[self._qpos_adr + 4]
        qy   = qpos[self._qpos_adr + 5]
        qz   = qpos[self._qpos_adr + 6]

        vx_mj = qvel[self._qvel_adr + 0]
        vy_mj = qvel[self._qvel_adr + 1]
        vz_mj = qvel[self._qvel_adr + 2]

        pos_ned = np.array([x_mj, y_mj, -z_mj])
        vel_ned = np.array([vx_mj, vy_mj, -vz_mj])
        euler = quat_to_euler(np.array([qw, qx, qy, qz]))

        return {
            "pos":   pos_ned,
            "vel":   vel_ned,
            "quat":  np.array([qw, qx, qy, qz]),
            "euler": euler,
            "t":     self.data.time,
        }

    def get_altitude(self):
        return float(self.data.qpos[self._qpos_adr + 2])

    def set_vehicle_pos(self, x, y):
        jx_adr = self.model.jnt_qposadr[self._veh_jnt_x]
        jy_adr = self.model.jnt_qposadr[self._veh_jnt_y]
        self.data.qpos[jx_adr] = x - 1.5
        self.data.qpos[jy_adr] = y
        mujoco.mj_forward(self.model, self.data)

    def get_vehicle_pos(self):
        xpos = self.data.xpos[self._vehicle_id]
        return np.array([xpos[0], xpos[1]])
