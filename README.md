# Introduction_to_drones_project-22AIE448
# Autonomous Landing of Quadrotor UAV
## using AprilTags Vision Positioning
## and Quaternion-based Attitude Representation


## 1. Introduction and Motivation

Autonomous landing is one of the most critical and difficult tasks for a quadrotor UAV. In many real-world situations (between tall buildings, indoors, under trees, or in GPS-denied environments), the GPS signal becomes weak or completely unavailable. In such cases, the drone cannot rely on satellite navigation for accurate landing.
Vision-based methods provide a practical solution. By mounting a downward-facing camera on the drone and placing known visual markers (AprilTags) on the landing platform, the drone can calculate its relative position and orientation in real time and land accurately without GPS.
This project is based on the research paper “UAV Autonomous Landing Technology Based on AprilTags Vision Positioning Algorithm” (Li et al., 2019). We take the core idea of the paper and improve it by introducing Unit Quaternions and the Sandwich Operator for attitude representation, which is a direct application of the mathematical concepts taught in our course 22AIE448.

