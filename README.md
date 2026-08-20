
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

### 1. Camera–Image Plane Coordinate Conversion

$$
u = \frac{x}{dx} + u_0 \qquad v = \frac{y}{dy} + v_0
$$

| Symbol | Meaning |
|---|---|
| `x, y` | Coordinates on the physical image plane (mm) |
| `dx, dy` | Physical size of one pixel (mm/pixel) |
| `u, v` | Pixel coordinates in the digital image |
| `u₀, v₀` | Principal point — optical centre of the camera |

**Explanation:** The camera sensor records the light on a continuous physical plane, but a digital image is stored as a discrete pixel grid. This equation converts real-world image-plane coordinates into pixel coordinates (and vice-versa), which is the first step before any pose estimation can happen — the AprilTag corner detector operates in pixel space, but geometric pose math needs metric image-plane coordinates.

---

### 2. Elementary Rotation Matrix (Roll, about X-axis)

$$
R_x(\phi) =
\begin{bmatrix}
1 & 0 & 0 \\
0 & \cos\phi & -\sin\phi \\
0 & \sin\phi & \cos\phi
\end{bmatrix}
$$

**Explanation:** This is the standard rotation matrix for a rotation of angle `φ` about the X-axis. Similar matrices `R_y(θ)` and `R_z(ψ)` exist for the Y and Z axes. Each encodes how a vector's coordinates change under a rotation about a single axis.

### 3. Combined Orientation (Euler Angle Composition)

$$
R = R_z(\psi)\, R_y(\theta)\, R_x(\phi)
$$

**Explanation:** The base paper represents the tag's full 3D orientation relative to the camera as a single 3×3 rotation matrix `R`, built by multiplying the three elementary rotations in Z-Y-X order (yaw–pitch–roll). This is the classical Euler-angle approach to attitude representation — and it's exactly the stage our project replaces with quaternions.

---

### 4. Rotation Matrix → Unit Quaternion Conversion

$$
q_0 = \frac{1}{2}\sqrt{1 + r_{11} + r_{22} + r_{33}}
$$

$$
q_1 = \frac{r_{32} - r_{23}}{4q_0} \qquad
q_2 = \frac{r_{13} - r_{31}}{4q_0} \qquad
q_3 = \frac{r_{21} - r_{12}}{4q_0}
$$

| Symbol | Meaning |
|---|---|
| `r_ij` | Element in row `i`, column `j` of rotation matrix `R` |
| `q₀` | Scalar (real) part of the quaternion |
| `q₁, q₂, q₃` | Vector (imaginary) part of the quaternion |

**Explanation:** This is Shepperd's method — the standard formula for converting a rotation matrix into a unit quaternion `q = (q₀, q₁, q₂, q₃)`, where `|q| = 1`. This is the step our project *adds* on top of the base paper's pipeline, giving a 4-parameter, singularity-free representation of the same orientation the matrix encodes with 9 (redundant) parameters.

---

### 5. Sandwich Operator (Quaternion Vector Rotation)

$$
L_q(v) = q\, v\, q^{*}
$$

Expanded form:

$$
L_q(v) = (q_0^2 - |\vec{q}|^2)\,v \;+\; 2q_0(\vec{q} \times v) \;+\; 2(\vec{q} \cdot v)\,\vec{q}
$$

| Symbol | Meaning |
|---|---|
| `q` | Unit quaternion representing the rotation |
| `q*` | Conjugate of `q` |
| `v` | Vector being rotated (here, the tag's position vector) |
| `q⃗` | Vector part `(q₁, q₂, q₃)` of the quaternion |

**Explanation:** The Sandwich Operator rotates a vector `v` by "sandwiching" it between a quaternion `q` and its conjugate `q*`. In our project, this replaces the base paper's rotation-matrix multiplication for transforming the AprilTag's position from the camera frame into the drone's body frame — giving a continuous, gimbal-lock-free transform.

---

### 6. PID Velocity Control

$$
V = K_p \cdot e \;+\; K_i \int e\, dt \;+\; K_d \cdot (e - e_{prev})
$$

**Gains used in the base paper:** `Kp = 0.20`, `Ki = 0.03`, `Kd = 0.35`

| Term | Meaning |
|---|---|
| `e` | Position error (Desired − Current position) |
| `Kp · e` | Proportional term — reacts to the current error |
| `Ki ∫e dt` | Integral term — corrects accumulated past error |
| `Kd(e − e_prev)` | Derivative term — dampens based on rate of change of error |
| `V` | Output velocity command sent to the drone |

**Explanation:** The position error computed from AprilTag pose estimation is fed into three independent PID controllers — one each for the X, Y, and Z axes. Each controller converts its axis's position error into a velocity command, which is what actually drives the drone's motors toward the landing target.

---

### 7. Position Error Computation

$$
\text{Error} = \text{Desired Position} - \text{Current Position (from AprilTag)}
$$

**Explanation:** This is the raw signal feeding the PID loop. It has three independent components — X-error and Y-error (horizontal offset from the tag centre) and Z-error (height above the landing platform) — each tracked and corrected by its own PID controller.



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


