
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



### 3.3  Base paper link:
[IEEE Xplore](https://ieeexplore.ieee.org/document/8865757) 

### 4.  Complete System Pipeline

<h2 align="center">Fig 4.1</h2>

<p align="center">
<img width="700" height="800" alt="image" src="https://github.com/user-attachments/assets/8f361a3d-0164-447e-81aa-852b44e49711" />
</p>

### 5.Methodology

### 6.Methodology Block Diagram

### 7.Results 

### 8.Conclusion

<h3 align="center">Table 8.1</h3>

| # | Reference | Link |
|---|---|---|
| 1 | Z. Li, Y. Chen, H. Lu, H. Wu, L. Cheng, "UAV Autonomous Landing Technology Based on AprilTags Vision Positioning Algorithm," CCC 2019 | [IEEE Xplore](https://ieeexplore.ieee.org/document/8865757) |
| 2 | E. Olson, "AprilTag: A Robust and Flexible Visual Fiducial System," ICRA 2011 | [IEEE Xplore](https://ieeexplore.ieee.org/document/5979561) · [Free PDF (UMich)](https://april.eecs.umich.edu/pdfs/olson2011a.pdf) |
| 3 | E. Fresk, G. Nikolakopoulos, "Full Quaternion Based Attitude Control for a Quadrotor," ECC 2013 | [Google Scholar](https://scholar.google.com/scholar?q=Full+Quaternion+Based+Attitude+Control+for+a+Quadrotor+Fresk+Nikolakopoulos) |
| 4 | D. Falanga, A. Zanchettin, A. Simovic, J. Delmerico, D. Scaramuzza, "Vision-Based Autonomous Quadrotor Landing on a Moving Platform," SSRR 2017 | [IEEE Xplore](https://ieeexplore.ieee.org/document/8088164) · [Free PDF (UZH RPG)](http://rpg.ifi.uzh.ch/docs/SSRR17_Falanga.pdf) |


