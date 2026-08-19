"""mujoco_3d_simulation.py — High-Fidelity 3D MuJoCo Autonomous Landing Simulation.

Features:
  - Native OpenGL 3D MuJoCo physics viewport
  - Direct hardware offscreen camera rendering with composite AprilTag texture
  - Realistic aerodynamics: motor lag, ground effect cushioning, Dryden wind gusts
  - Detailed DJI Matrice 100 3D body + UGV vehicle model

Usage:
    python mujoco_3d_simulation.py                 # Static Landing (Exp 1)
    python mujoco_3d_simulation.py --mode dynamic  # Dynamic Tracking (Exp 2)
    python mujoco_3d_simulation.py --wind          # Enable wind gusts

Controls in 3D Viewer & Camera Window:
    - Left Click + Drag  : Orbit camera in 3D
    - Right Click + Drag : Zoom in/out
    - Middle Click + Drag: Pan camera
    - [SPACE]            : Pause / Resume
    - [R]                : Reset to initial 12m altitude
    - [G]                : Toggle wind gust disturbance
    - [Q] / [ESC]        : Exit
"""

import os
import sys
import time
import argparse
import numpy as np
import cv2
import mujoco
import mujoco.viewer

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from envs.quad_env import QuadEnv, attitude_from_accel, MASS, HOVER_THRUST
from vision.camera_sim import make_ground_image, render_frame, Camera, LARGE_SIZE, SMALL_SIZE
from vision.apriltag_detect import ApriltTagMeasure
from control.pid_controller import DroneController


def run_3d_sim(mode="static", speed=1.0, wind=False):
    print("\n" + "=" * 70)
    print(f"  LAUNCHING HIGH-FIDELITY MUJOCO 3D PHYSICS SIMULATION")
    print(f"  Mode: {mode.upper()} | Wind Gusts: {'ON' if wind else 'OFF'}")
    print("  Base Paper: Li et al. (Chinese Control Conference 2019)")
    print("=" * 70)
    print("\n  Interactive Controls:")
    print("    - Mouse Left-Click + Drag  : Orbit 3D camera")
    print("    - Mouse Right-Click + Drag : Zoom in/out")
    print("    - Mouse Middle-Click       : Pan camera")
    print("    - [SPACE]                  : Pause / Resume")
    print("    - [R]                      : Reset drone")
    print("    - [G]                      : Toggle wind gust")
    print("    - [Q] / [ESC]              : Exit\n")

    rng    = np.random.default_rng(7)
    env    = QuadEnv(enable_camera_render=True)
    ctrl   = DroneController(use_pid=True)
    meas   = ApriltTagMeasure()
    ground = make_ground_image(rng)

    if mode == "static":
        start_pos = (1.2, -0.8, -12.0)
    else:
        start_pos = (0.3, -0.2, -3.54)

    env.reset(pos_ned=start_pos)
    ctrl.reset()
    meas.reset()

    wind_active = wind

    cv2.namedWindow("Onboard AprilTag Camera Feed", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Onboard AprilTag Camera Feed", 540, 430)

    with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
        viewer.cam.distance = 18.0
        viewer.cam.elevation = -22.0
        viewer.cam.azimuth = 135.0
        viewer.cam.lookat[:] = [0.6, -0.4, 4.0]

        paused = False
        landed = False
        t = 0.0
        last_cam_t = -1.0
        last_meas = (False, 0.0, 0.0, 0.0, None)
        acc = np.zeros(3)
        tracking_phase = "TRACKING"

        dt_step = QuadEnv.DT_CTRL

        while viewer.is_running():
            step_start = time.time()

            if not paused and not landed:
                state    = env.get_state()
                pos      = state["pos"]
                vel      = state["vel"]
                alt_true = -pos[2]
                att      = attitude_from_accel(acc)

                # Ground vehicle motion
                if mode == "dynamic":
                    veh_x = 0.8 * np.sin(0.20 * t)
                    veh_y = 0.5 * np.sin(0.28 * t + np.pi/4)
                    pad_pos = (veh_x, veh_y)
                    env.set_vehicle_pos(veh_x, veh_y)
                else:
                    pad_pos = (0.0, 0.0)

                # Camera rendering ~25 Hz
                if t - last_cam_t >= 0.04 or t < 0.01:
                    cam_frame = render_frame(pos, att, rng, ground, pad_pos)
                    lock, alt_m, east_m, north_m, corners = meas.detect(cam_frame, att, alt_true)
                    last_meas = (lock, alt_m, east_m, north_m, corners)
                    last_cam_t = t

                lock, alt_m, east_m, north_m, corners = last_meas

                # Controller update
                if mode == "static":
                    fx, fy, fz, vz_cmd = ctrl.update(lock, alt_m, east_m, north_m, vel, t)
                    if (alt_true < 0.15) or (ctrl.is_landed(alt_m if lock else alt_true, vel) and alt_true < 0.25):
                        landed = True
                        print(f"\n  [OK] TOUCHDOWN on static pad at t={t:.2f}s | Final: (N:{pos[0]:+.3f}, E:{pos[1]:+.3f}, Alt:{alt_true:.3f}m)")
                else:
                    offset_x = pos[0] - pad_pos[0]
                    offset_y = pos[1] - pad_pos[1]
                    if tracking_phase == "TRACKING":
                        fx, fy, fz, _ = ctrl.update(lock, 3.54, east_m, north_m, vel, t)
                        err_alt = 3.54 - alt_true
                        fz = float(np.clip(HOVER_THRUST + 2.5 * err_alt * MASS, 0.0, 75.0))
                        if lock and np.hypot(offset_x, offset_y) < 0.20 and t > 15.0:
                            tracking_phase = "LANDING"
                            ctrl.reset()
                            print(f"\n  t={t:.2f}s | Target centered -> Initiating descent")
                    else:
                        fx, fy, fz, vz_cmd = ctrl.update(lock, alt_m, east_m, north_m, vel, t)
                        if (alt_true < 0.15) or (ctrl.is_landed(alt_m if lock else alt_true, vel) and alt_true < 0.25):
                            landed = True
                            print(f"\n  [OK] TOUCHDOWN on moving vehicle at t={t:.2f}s | Offset: dX={offset_x:+.3f}m, dY={offset_y:+.3f}m)")

                acc = np.array([fx / MASS, fy / MASS, (HOVER_THRUST - fz) / MASS])
                env.step(fx_north=fx, fy_east=fy, fz_up=fz, apply_wind=wind_active)

                t += dt_step

            # Sync 3D viewer
            viewer.sync()

            # Render camera HUD overlay
            vis_cam = cam_frame.copy() if 'cam_frame' in locals() else ground.copy()
            lock, alt_m, east_m, north_m, corners = last_meas
            if lock and corners is not None:
                cv2.polylines(vis_cam, [corners.astype(np.int32)], True, (0, 255, 110), 2)
                ctr = corners.mean(axis=0).astype(int)
                cv2.circle(vis_cam, (ctr[0], ctr[1]), 5, (0, 255, 110), -1)
                cv2.line(vis_cam, (ctr[0], ctr[1]), (int(Camera.CX), int(Camera.CY)), (0, 255, 110), 1)
                hud_tag = f"LOCKED (Alt: {alt_m:.2f}m | dN: {north_m:+.2f}m | dE: {east_m:+.2f}m)"
                hud_col = (0, 255, 110)
            else:
                hud_tag = "SEARCHING / NO TARGET"
                hud_col = (60, 60, 240)

            cv2.putText(vis_cam, hud_tag, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.52, hud_col, 2, cv2.LINE_AA)
            tag_size_str = "0.20m (Small Tag)" if (alt_m if lock else alt_true) < 2.0 else "0.80m (Large Tag)"
            cv2.putText(vis_cam, f"Tag Type: {tag_size_str} | PID: Kp=0.20 Ki=0.03 Kd=0.35",
                        (12, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (210, 220, 230), 1, cv2.LINE_AA)
            cv2.putText(vis_cam, f"Sim Time: {t:5.2f}s | Wind: {'ON' if wind_active else 'OFF'}",
                        (12, 78), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (140, 210, 245), 1, cv2.LINE_AA)

            if landed:
                cv2.putText(vis_cam, "LANDED [TOUCHDOWN OK]", (12, 115),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 110), 2, cv2.LINE_AA)

            cv2.imshow("Onboard AprilTag Camera Feed", vis_cam)
            k = cv2.waitKey(1) & 0xFF
            if k in (ord('q'), ord('Q'), 27):
                break
            elif k == 32:
                paused = not paused
                print("  [PAUSED]" if paused else "  [RESUMED]")
            elif k in (ord('g'), ord('G')):
                wind_active = not wind_active
                print(f"  [WIND GUSTS] {'ENABLED' if wind_active else 'DISABLED'}")
            elif k in (ord('r'), ord('R')):
                env.reset(pos_ned=start_pos)
                ctrl.reset()
                meas.reset()
                landed = False
                t = 0.0
                tracking_phase = "TRACKING"
                print("  [RESET] Drone reset to initial altitude")

            elapsed = time.time() - step_start
            sleep_time = (dt_step / speed) - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    cv2.destroyAllWindows()
    print("\n3D Simulation finished.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="High-Fidelity MuJoCo 3D Simulation")
    parser.add_argument("--mode", choices=["static", "dynamic"], default="static",
                        help="3D simulation mode: static landing (Exp 1) or dynamic tracking (Exp 2)")
    parser.add_argument("--speed", type=float, default=1.0, help="Simulation playback speed multiplier")
    parser.add_argument("--wind", action="store_true", help="Enable stochastic wind gust turbulence")
    args = parser.parse_args()

    run_3d_sim(mode=args.mode, speed=args.speed, wind=args.wind)
