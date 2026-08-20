
<p align="center">
  <img width="700" height="90" alt="image" src="https://github.com/user-attachments/assets/a9600b74-0941-4c72-833f-af076c559691" />
</p>

<h1 align="center">Introduction to Data-Driven Control of Drones - 22AIE448</h1>

<h1 align="center">Autonomous Landing of Quadrotor UAV Using AprilTags Vision Positioning and Quaternion-based Attitude Representation </h1>

### *Team Members*
Police Aryan - CB.SC.U4AIE24241\
Gayathri Vaishnav - CB.SC.U4AIE24337\
Rohit Vardhan  - CB.SC.U4AIE24231\
Sai Reddy - CB.SC.U4AIE24205\
Sairi Manvik - CB.SC.U4AIE24352

## 1. Abstract

Autonomous landing remains one of the most challenging phases of quadrotor UAV flight, particularly in GPS-denied environments such as indoor spaces, dense urban canyons, and forested or covered terrain, where satellite navigation becomes unreliable or entirely unavailable. This project presents an onboard, vision-based autonomous landing and dynamic tracking system for a quadrotor UAV that relies solely on a downward-facing monocular camera and multi-scale AprilTag fiducial markers for real-time relative pose estimation. Building on the foundational architecture proposed by Li et al. (2019), the system employs two coplanar AprilTags of different sizes to maintain accurate positioning across the full descent  from high altitude to touchdown  and a classical PID controller to convert position error into stable velocity commands.

The central contribution of this work is the replacement of the conventional rotation-matrix-based attitude representation with a **Unit Quaternion and Sandwich Operator** formulation, directly applying the quaternion theory covered in course 22AIE448. This substitution eliminates the risk of gimbal lock inherent to Euler-angle-based representations, provides a continuous and numerically stable method for transforming position vectors between the camera and body frames, and improves control smoothness during aggressive banking or turning maneuvers while tracking a moving platform. The resulting system is designed to achieve sub-1% landing error and robust tracking performance on both static and moving landing platforms, validating the practical benefit of quaternion-based attitude representation in a real-world robotics application.

**Keywords:** UAV autonomous landing, AprilTag, visual positioning, unit quaternion, Sandwich Operator, PID control, GPS-denied 


## 2. Introduction

Autonomous landing is widely regarded as one of the most safety-critical and technically demanding phases of unmanned aerial vehicle (UAV) operation, since even small errors in position or orientation during the final descent can result in a failed landing, damage to the vehicle, or harm to nearby people and property. While GPS-based navigation performs reliably in open outdoor environments, its accuracy degrades sharply or fails outright in scenarios common to real-world deployment  flight between tall buildings where signals are reflected and multipath errors accumulate, indoor operation where satellite visibility is entirely blocked, movement beneath tree canopies where foliage attenuates the signal, and other GPS-denied settings encountered in warehouses, disaster zones, or dense urban canyons. In these conditions, a quadrotor cannot depend on satellite positioning to execute a safe, precise landing, creating a clear and practical need for an alternative, self-contained sensing approach that does not rely on external infrastructure. Vision-based positioning offers exactly this kind of practical and low-cost solution: by mounting a downward-facing monocular camera on the UAV and placing known fiducial markers in this case, AprilTags on the landing platform, the drone can compute its relative position and orientation with respect to the target in real time, entirely onboard, using only the visual geometry of the tag as seen by the camera. This approach removes the dependency on GPS altogether while remaining computationally lightweight enough to run in real time on modest onboard hardware, making it well suited to small and medium-sized quadrotors with limited payload and processing capacity. Building on this idea, the present project takes as its architectural foundation the method proposed by Li et al. in "UAV Autonomous Landing Technology Based on AprilTags Vision Positioning Algorithm" (2019), which demonstrated a complete onboard pipeline for AprilTag-based autonomous landing using a dual-tag detection strategy  a large tag for reliable detection at high altitude and a small tag for precise positioning close to the ground combined with a classical PID velocity controller that converts the estimated position error into smooth, stable motor commands. This project adopts that pipeline as its starting point, reusing the camera-based tag detection stage, the dual-tag altitude-dependent switching logic, and the underlying PID control structure, while extending the attitude representation stage of the system with unit quaternions and the Sandwich Operator, as detailed in the sections that follow.

### 2.1 Our Contribution

The base paper represents drone orientation using a 3×3 **rotation matrix**, derived from sequential Euler-angle rotations. While functionally adequate under normal flight conditions, this representation is known to suffer from **gimbal lock**  a singularity that can cause a loss of one rotational degree of freedom during large-angle maneuvers and is computationally redundant compared to more compact alternatives.

In this project, we address this limitation by introducing **Unit Quaternions** and the **Sandwich Operator** to represent and transform orientation, replacing the rotation-matrix stage of the original pipeline. This is a direct, applied extension of the quaternion theory taught in course *22AIE448 – Introduction to Drones*, and forms the core technical contribution of this work. Specifically:

- The rotation matrix obtained from AprilTag pose estimation is converted into a unit quaternion, avoiding the singularities associated with Euler-angle representations.
- The Sandwich Operator is used to transform the tag's position vector from the camera frame into the drone's body frame in a numerically stable, continuous manner.
- This improves control smoothness and robustness, particularly during banking or turning maneuvers while tracking a moving landing platform.

By combining the proven vision-based landing architecture of the base paper with a quaternion-based attitude formulation, this project demonstrates a practical, singularity-free improvement to an established autonomous landing method.

### 3.  Base paper/ Literature Survey 
<h2 align="center">Table 3.1</h2>

| # | Reference | Authors / Year | Title    | Key Methodology | Why Used in Our Project | Limitations |
|---|---|---|---|---|---|---|
| 1 | Base Paper | Z. Li, Y. Chen, H. Lu, H. Wu, L. Cheng (2019) | "UAV Autonomous Landing Technology Based on AprilTags Vision Positioning Algorithm," 38th Chinese Control Conference (CCC) | Monocular camera + dual-sized AprilTags for pose estimation; classical PID controller for velocity commands; tested on static and moving platforms | This is our **primary base paper** we reuse its full pipeline (dual-tag detection, pose estimation, PID control) and extend the attitude representation stage with quaternions | Uses rotation matrices (Euler-based) for attitude susceptible to gimbal lock; fixed PID gains; single test environment, no robustness analysis |
| 2 | Fiducial Marker | E. Olson (2011) | "AprilTag: A Robust and Flexible Visual Fiducial System," IEEE ICRA | Introduces the AprilTag fiducial marker family 2D barcode-like tags with robust detection, sub-pixel corner localization, and low false-positive rate even under occlusion/lighting variation | Foundational reference for the **marker system itself** explains why AprilTag was chosen over other fiducials (accuracy, open-source, low computational cost) for our detection stage | Detection accuracy still degrades at extreme viewing angles or very low resolution; no built-in multi-scale/multi-tag switching strategy (added later by other works) |
| 3 | Attitude Representation | E. Fresk, G. Nikolakopoulos (2013) | "Full Quaternion Based Attitude Control for a Quadrotor," European Control Conference (ECC) | Formulates full nonlinear quadrotor attitude control directly in unit-quaternion space, avoiding Euler-angle linearization and singularities | Directly supports our **core contribution** justifies replacing the base paper's rotation-matrix attitude representation with unit quaternions + Sandwich Operator for singularity-free, smoother control | Focuses on attitude control law design, not vision-based pose estimation; doesn't address marker detection or landing-specific error dynamics |
| 4 | Moving-Platform Landing | D. Falanga, A. Zanchettin, A. Simovic, J. Delmerico, D. Scaramuzza (2017) | "Vision-Based Autonomous Quadrotor Landing on a Moving Platform," IEEE SSRR | Onboard vision pipeline with marker detection, relative velocity estimation, and adaptive control law for landing on platforms moving at varying speeds | Supports our **dynamic/moving-platform validation** goal provides additional benchmarking methodology and control strategy references beyond the base paper's simpler PID approach | Requires higher onboard computational power for real-time relative velocity estimation; assumes platform motion is roughly planar and bounded in speed |

### 3.2  Base paper math:

## Mathematical Foundation of the Base Paper

## Base Paper Equations 

### 1. Pixel Conversion (Camera → Digital Image)

$$
u = \frac{x}{dx} + u_0 \qquad v = \frac{y}{dy} + v_0
$$

 A camera sensor sees the world as a continuous picture, but a computer stores it as a grid of pixels (like graph paper). This equation just says: "take the real position on the sensor and tell me which pixel box it lands in."

| Variable | What it is | Why it's needed |
|---|---|---|
| `x, y` | Where a point actually is on the camera sensor (in millimeters) | This is the "real" physical location before we turn it into pixels |
| `dx, dy` | How big one pixel is (in millimeters) | Tells us how many pixels fit into one millimeter — this is what does the mm → pixel conversion |
| `u, v` | The pixel address of that point (like column, row) | This is what the computer actually works with — an image is just numbers in a pixel grid |
| `u0, v0` | The pixel at the exact center of the image | Cameras don't always have their center at pixel (0,0), so we shift by this amount to correct for that |

---

### 2. One-Axis Rotation (Roll)

$$
R_x(\phi) =
\begin{bmatrix}
1 & 0 & 0 \\
0 & \cos\phi & -\sin\phi \\
0 & \sin\phi & \cos\phi
\end{bmatrix}
$$

 Imagine tilting a tray left-to-right — that's roll. This matrix is just a "tilt calculator": you give it an angle, and it tells you how every point on the tray moves.

| Variable | What it is | Why it's needed |
|---|---|---|
| `φ` (phi) | The roll angle — how much the drone/tag is tilted sideways | Without this, we can't tell how the object rotated around one axis |
| `R_x(φ)` | The 3×3 matrix itself | It's a reusable "rotation machine" — multiply it with a point and it spins that point by angle φ |

---

### 3. Full 3D Rotation (Combining Roll, Pitch, Yaw)

$$
R = R_z(\psi)\, R_y(\theta)\, R_x(\phi)
$$

 A drone doesn't just tilt in one direction — it rolls, pitches (nose up/down), and yaws (spins left/right) all at once. So we just chain three simple tilt calculators together to get the *full* rotation.

| Variable | What it is | Why it's needed |
|---|---|---|
| `ψ` (psi) | Yaw angle — spinning left/right | Captures rotation around the vertical axis |
| `θ` (theta) | Pitch angle — nose up/down | Captures forward/backward tilt |
| `φ` (phi) | Roll angle — tilting side to side | Captures sideways tilt |
| `R` | The combined 3×3 rotation matrix | This single matrix now fully describes the tag's orientation relative to the camera |

---

### 4. Turning the Rotation Matrix into a Quaternion

$$
q_0 = \frac{1}{2}\sqrt{1 + r_{11} + r_{22} + r_{33}}
$$

$$
q_1 = \frac{r_{32} - r_{23}}{4q_0}, \quad
q_2 = \frac{r_{13} - r_{31}}{4q_0}, \quad
q_3 = \frac{r_{21} - r_{12}}{4q_0}
$$

 The rotation matrix above works fine, but it uses 9 numbers to describe something that really only needs 4. This equation squeezes that same rotation into just 4 numbers — a quaternion — which is lighter to compute with and doesn't get "stuck" the way the matrix can.

| Variable | What it is | Why it's needed |
|---|---|---|
| `r_ij` | A single number sitting in row `i`, column `j` of the rotation matrix `R` | These are just the raw ingredients — we're reading values straight out of the matrix from step 3 |
| `q0` | The "how much rotation" part of the quaternion | This is the main number that tells you the size of the rotation |
| `q1, q2, q3` | The "which direction" part of the quaternion (a 3D arrow) | Together with q0, this arrow + amount fully describes the same rotation as the matrix — just more compactly |

---

### 5. Rotating a Vector with a Quaternion (Sandwich Operator)

$$
L_q(v) = q\, v\, q^{*}
$$

 Think of `v` as a small arrow (like "the tag is 2 meters that way"). To rotate that arrow using a quaternion, you "sandwich" it — multiply the quaternion on one side and its mirror-image (conjugate) on the other. What comes out is the same arrow, just rotated.

| Variable | What it is | Why it's needed |
|---|---|---|
| `q` | The quaternion describing the rotation (from step 4) | This is the "rotation instruction" |
| `q*` | The conjugate of `q` — basically `q` flipped/reversed | Needed to "undo" the extra rotation math and leave you with a clean, correctly rotated vector |
| `v` | The vector being rotated — here, the tag's position | This is the actual thing we care about moving — we want to know where the tag is *relative to the drone*, not the camera |
| `L_q(v)` | The final, rotated vector | This is the answer — the tag's position now expressed in the drone's own frame of reference |

---

### 6. PID Controller (Turning Error into Speed Commands)

$$
V = K_p \cdot e \;+\; K_i \int e\, dt \;+\; K_d \cdot (e - e_{prev})
$$

 This is just "how fast should the drone move to fix its position error?" It looks at three things: how far off you are *right now*, how far off you've *been* over time, and how *quickly* the error is changing — and blends all three into one speed command.

| Variable | What it is | Why it's needed |
|---|---|---|
| `e` | The current position error (how far the drone is from where it should be) | This is the main signal — no error means no correction needed |
| `Kp · e` | Proportional term — reacts to the error right now | Bigger error → bigger push. This is the main "push toward target" force |
| `Ki ∫e dt` | Integral term — adds up all past error over time | Corrects for small, persistent errors that proportional alone can't fully fix (like drift) |
| `Kd(e − e_prev)` | Derivative term — looks at how fast the error is changing | Acts like a brake — stops the drone from overshooting or oscillating |
| `Kp, Ki, Kd` | Tuning knobs (gains) for each term | These control how aggressive or gentle each correction is — set by testing, not calculated |
| `V` | The final speed command sent to the drone | This is the actual output — what makes the drone actually move |

---

### 7. Position Error

$$
\text{Error} = \text{Desired Position} - \text{Current Position}
$$

 This is the simplest equation here — it's just "where you want to be" minus "where you actually are." That difference is what every PID controller above is trying to shrink to zero.

| Variable | What it is | Why it's needed |
|---|---|---|
| Desired Position | Where the drone should be (usually right above the tag) | This is the target — set by the landing task itself |
| Current Position | Where the drone actually is, as measured by the AprilTag | This comes from the vision pipeline — it's our best real-time estimate of the truth |
| Error | The gap between the two | This single number (per axis) is what drives the entire PID control loop |


### 3.3  Base paper link:
[IEEE Xplore](https://ieeexplore.ieee.org/document/8865757) 

### 4.  Complete System Pipeline

<h2 align="center">Fig 4.1</h2>

<p align="center">
<img width="700" height="800" alt="image" src="https://github.com/user-attachments/assets/8f361a3d-0164-447e-81aa-852b44e49711" />
</p>

### 5.Methodology

## Methodology

Our implementation is a full physics-based simulation of the base paper's pipeline, built in MuJoCo, with three main modules working in a closed loop at each control step:

1. **Physics simulation (`envs/quad_env.py`)** — A high-fidelity MuJoCo model of a DJI M100-class quadrotor (3.5 kg) is stepped at 500 Hz internally (`DT_SIM = 0.002s`) and controlled at 50 Hz (`DT_CTRL = 0.02s`). It includes motor lag, ground-effect lift, and Dryden wind turbulence. The drone's true orientation is read directly from MuJoCo as a quaternion `[qw, qx, qy, qz]` and converted to Euler angles for the vision model.

2. **Vision simulation (`vision/camera_sim.py`, `vision/apriltag_detect.py`)** — A synthetic downward camera (640×480, FX=FY=650) renders the ground, landing pad, and a perspective-warped AprilTag (`tag36h11`) into an image, with Gaussian blur and pixel noise added for realism. The tag physically switches between a large (0.80 m) and small (0.20 m) marker at a 1.8 m altitude threshold. On the detection side, `pupil_apriltags` locates the tag in the image, and OpenCV's `solvePnP` recovers the tag's pose relative to the camera. The world-frame altitude and lateral offsets are then extracted using the world→body and body→camera rotation matrices, and smoothed with an exponential moving average (EMA) filter.

3. **Control (`control/pid_controller.py`)** — A finite-state controller (`SEARCH → APPROACH → DESCEND_HIGH → DESCEND_LOW → LAND`) selects a target descent speed for the current phase and feeds the lateral (X, Y) position errors into two independent PID loops. Vertical velocity is tracked with a proportional velocity loop. The resulting lateral/vertical accelerations are converted into thrust forces (with acceleration and thrust clipping for realism) and applied back into the MuJoCo physics step, closing the loop.

This differs from the base paper in two important ways: pose estimation uses OpenCV's iterative PnP solver instead of a closed-form rotation-matrix decomposition, and the controller is phase-aware (it changes behaviour by altitude/lock state) rather than a single flat PID loop — closer to how a real onboard autopilot would behave.

## Equations Used in the Implementation

### 1. Pinhole Camera Projection

Each 3D world point is projected into image pixel coordinates:

```math
p_{cam} = R_{cb} \, R_{wb}(p_{world} - p_{drone})
```

```math
\begin{bmatrix} u \\ v \end{bmatrix} = \frac{1}{z_{cam}}
\begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \end{bmatrix} p_{cam}
```

| Symbol | Meaning |
|---|---|
| `R_wb` | World → body rotation matrix (from roll, pitch) |
| `R_cb` | Body → camera rotation matrix (fixed downward-facing mount) |
| `f_x, f_y` | Focal lengths (650 px) |
| `c_x, c_y` | Principal point (image centre, 320, 240) |

This is the forward model used to *render* the synthetic camera image of the tag and landing pad.

---

### 2. World-to-Body and Body-to-Camera Rotations

```math
R_{wb} =
\begin{bmatrix} \cos\theta & 0 & \sin\theta \\ 0 & 1 & 0 \\ -\sin\theta & 0 & \cos\theta \end{bmatrix}
\begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos\phi & -\sin\phi \\ 0 & \sin\phi & \cos\phi \end{bmatrix}
```

```math
R_{cb} =
\begin{bmatrix} 0 & 1 & 0 \\ -1 & 0 & 0 \\ 0 & 0 & 1 \end{bmatrix}
```

`R_wb` combines pitch (`θ`) and roll (`φ`) into a single rotation; `R_cb` is a fixed axis realignment from the drone body frame into the downward-facing camera frame.

---

### 3. Pose Recovery via Perspective-n-Point (PnP)

Given the 4 detected tag corners in the image and their known real-world corner positions, OpenCV solves for the rotation vector `r` and translation vector `t` that minimize reprojection error:

```math
\arg\min_{r,\,t} \sum_{i=1}^{4} \left\| \text{proj}(X_i,\, r,\, t) - x_i \right\|^2
```

| Symbol | Meaning |
|---|---|
| `X_i` | Known 3D tag corner in the tag's own frame |
| `x_i` | Corresponding detected 2D pixel coordinate |
| `proj(·)` | Pinhole projection function |
| `r, t` | Rotation vector and translation vector of the tag relative to the camera |

This replaces the closed-form rotation-matrix decomposition used in the base paper with an iterative optimization — standard practice in real vision pipelines.

---

### 4. Altitude and Lateral Offset Extraction

```math
\text{offset} = R_{wb}^{T} \left( R_{cb}^{T}\, t \right)
```

```math
\text{alt} = \text{offset}_z \qquad \text{east\_err} = \text{offset}_y \qquad \text{north\_err} = \text{offset}_x
```

The camera-frame translation vector `t` from PnP is rotated back through body and world frames to recover the drone's true altitude and lateral position error relative to the tag.

---

### 5. Exponential Moving Average (EMA) Measurement Filter

```math
\hat{x}_k = \alpha \, x_k + (1-\alpha)\, \hat{x}_{k-1}, \qquad \alpha = 0.55
```

Smooths noisy per-frame pose measurements `x_k` into a filtered estimate `\hat{x}_k`, reducing jitter from detection noise before the value reaches the controller.

---

### 6. Quaternion to Euler Angle Conversion

The drone's true attitude is read from MuJoCo as a unit quaternion `q = (w, x, y, z)` and converted to roll, pitch, yaw:

```math
\text{roll} = \arctan2\big(2(wx + yz),\; 1 - 2(x^2 + y^2)\big)
```

```math
\text{pitch} = \arcsin\big(2(wy - zx)\big)
```

```math
\text{yaw} = \arctan2\big(2(wz + xy),\; 1 - 2(y^2 + z^2)\big)
```

This is the quaternion-to-Euler conversion (the inverse operation of the base paper's rotation-matrix-to-quaternion step) — used here to feed Euler-based attitude into the vision projection model.

---

### 7. PID Axis Update (with Anti-Windup and Output Clamping)

```math
I_k = \text{clip}\big(I_{k-1} + e_k,\; -I_{max},\; I_{max}\big)
```

```math
v_{cmd} = K_p e_k + K_i I_k + K_d (e_k - e_{k-1})
```

```math
v_{cmd} = \text{clip}(v_{cmd},\; -v_{max},\; v_{max})
```

**Gains:** `Kp = 0.20, Ki = 0.03, Kd = 0.35` (lateral axes); `Kp = 0.25, Ki = 0.02, Kd = 0.30` (vertical axis)

Unlike the base paper's unbounded PID formula, the implementation clamps the integral term (anti-windup) and the final output (actuator saturation) — both necessary for physical realism in simulation.

---

### 8. Lateral Force Command

```math
a_x = K_{v,lat}(v_{x,cmd} - v_x), \qquad a_y = K_{v,lat}(v_{y,cmd} - v_y)
```

```math
\text{if } \|a\| > a_{max}: \quad a \leftarrow a \cdot \frac{a_{max}}{\|a\|}
```

```math
F_x = m\,a_x, \qquad F_y = m\,a_y
```

The PID output is a *velocity* command; a proportional velocity-tracking loop (`K_v,lat = 2.0`) converts it into acceleration, which is clipped to `MAX_ACC = 4.0 m/s²` and scaled to a force by drone mass `m = 3.5 kg`.

---

### 9. Vertical Thrust Command

```math
a_z = \text{clip}\big(K_{v,z}(v_{z,cmd} - v_z),\; -3.5,\; 6.0\big)
```

```math
F_{z,up} = \text{clip}\big(T_{hover} - m\,a_z,\; 0,\; 75\big), \qquad T_{hover} = mg
```

The total upward thrust is the hover thrust `T_hover = mg = 34.335 N` adjusted by the commanded vertical acceleration, clipped to stay within realistic rotor thrust limits (and reduced to 75% during the final `LAND` phase for a soft touchdown).

---

### 10. Landing Condition

```math
\text{landed} = (\text{alt} < 0.15\,\text{m}) \;\wedge\; (|v_z| < 0.35\,\text{m/s})
```

Landing is only declared once the drone is both low enough and moving slowly enough vertically — avoiding a false "landed" trigger mid-descent.
### 6.Methodology Block Diagram

<h2 align="center">Fig 6.1</h2>
<p align="center">
<img width="700" height="700" alt="image" src="https://github.com/user-attachments/assets/6e49ec3c-b354-4c10-9adc-d0728c1e1b22" />
</p>

### 7.Results 

### 8.Conclusion

<h3 align="center">Table 8.1</h3>

| # | Reference | Link |
|---|---|---|
| 1 | Z. Li, Y. Chen, H. Lu, H. Wu, L. Cheng, "UAV Autonomous Landing Technology Based on AprilTags Vision Positioning Algorithm," CCC 2019 | [IEEE Xplore](https://ieeexplore.ieee.org/document/8865757) |
| 2 | E. Olson, "AprilTag: A Robust and Flexible Visual Fiducial System," ICRA 2011 | [IEEE Xplore](https://ieeexplore.ieee.org/document/5979561) · [Free PDF (UMich)](https://april.eecs.umich.edu/pdfs/olson2011a.pdf) |
| 3 | E. Fresk, G. Nikolakopoulos, "Full Quaternion Based Attitude Control for a Quadrotor," ECC 2013 | [Google Scholar](https://scholar.google.com/scholar?q=Full+Quaternion+Based+Attitude+Control+for+a+Quadrotor+Fresk+Nikolakopoulos) |
| 4 | D. Falanga, A. Zanchettin, A. Simovic, J. Delmerico, D. Scaramuzza, "Vision-Based Autonomous Quadrotor Landing on a Moving Platform," SSRR 2017 | [IEEE Xplore](https://ieeexplore.ieee.org/document/8088164) · [Free PDF (UZH RPG)](http://rpg.ifi.uzh.ch/docs/SSRR17_Falanga.pdf) |


