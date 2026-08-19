
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

### 1. Introduction 

Autonomous landing is one of the most critical and difficult tasks for a quadrotor UAV. In many real-world situations (between tall buildings, indoors, under trees, or in GPS-denied environments), the GPS signal becomes weak or completely unavailable. In such cases, the drone cannot rely on satellite navigation for accurate landing.\
Vision-based methods provide a practical solution. By mounting a downward-facing camera on the drone and placing known visual markers (AprilTags) on the landing platform, the drone can calculate its relative position and orientation in real time and land accurately without GPS.\
This project is based on the research paper “UAV Autonomous Landing Technology Based on AprilTags Vision Positioning Algorithm” (Li et al., 2019). We take the core idea of the paper and improve it by introducing Unit Quaternions and the Sandwich Operator for attitude representation, which is a direct application of the mathematical concepts taught in our course 22AIE448.

### 2.  Base paper explanation

The base paper proposes an onboard solution for autonomous landing of a quadrotor using AprilTags. The key ideas of the paper are:\
•	A monocular camera is used to detect AprilTags placed on the landing platform.\
•	Two tags of different sizes are used – a large tag for high altitude and a small tag for low altitude.\
•	The relative position and orientation (pose) of the drone with respect to the tag are calculated using the AprilTag algorithm.\
•	A classical PID controller is used to control the  velocity of the drone.\
•	Experiments showed landing error less than 1% and good tracking performance on a moving platform.

### 2.1  Base paper math:



### 2.2  Base paper link:

### 3.  Complete System Pipeline
The full working of our system can be understood in the following steps:
1.	Image Capture: A downward-facing monocular camera continuously captures images of the ground.
2.	AprilTag Detection: The system detects the AprilTag(s) in the image and finds the four corner points.
3.	Pose Estimation: Using the known physical size of the tag and the camera calibration, the relative position and rotation matrix between the tag and the camera are calculated.
4.	Quaternion Conversion: The rotation matrix is converted into a Unit Quaternion.
5.	Frame Transformation: The Sandwich Operator is applied to express the position error in the drone’s body frame.
6.	PID Control: The body-frame position errors (X, Y, Z) are given to three PID controllers that generate velocity commands.
7.	Landing: When the horizontal error is small and the height is low enough, the drone starts the final landing manoeuvre.

### Methodology

### Methodology Block Diagram

### Project pipeline 

### Results 

### Conclusion

### 3.References

[1] Z. Li, Y. Chen, H. Lu, H. Wu and L. Cheng, “UAV Autonomous Landing Technology Based on AprilTags Vision Positioning Algorithm,” Proceedings of the 38th Chinese Control Conference, Guangzhou, China, July 2019, pp. 8148–8153.\
[2] E. Olson, “AprilTag: A robust and flexible visual fiducial system,” IEEE International Conference on Robotics and Automation (ICRA), 2011.\
[3] Course Notes on Quaternions, Introduction to Drones – 22AIE448, School of Artificial Intelligence, Amrita Vishwa Vidyapeetham (Dr. Sunil Kumar S and Prof. K. P. Soman), July 2026.


