# DIFFERENTIALDRIVER
Modular MATLAB–ROS2 project for a differential-drive robot. Generates smooth spline paths from 2D waypoints, creates time-parameterized trajectories, and tracks them using a feedback controller. Includes ROS2 integration, visualizations, and can be extended for real-time obstacle avoidance.
Differential Drive Trajectory Tracking – MATLAB + ROS2

FOR MORE INFORMATION READ THE ROBOT ASSIGNEMENT DOCUMENTATION.

This project implements path smoothing, trajectory generation, and feedback control for a TurtleBot3 differential-drive robot in MATLAB with ROS2 integration.
It converts discrete 2D waypoints into smooth trajectories and makes the robot follow them in simulation.

Setup Instructions

Clone or download this repository.

Open MATLAB and add all files to your MATLAB path (Add Folder and Subfolders).

Ensure ROS2 (Humble) is installed and accessible from WSL.

Launch the TurtleBot3 simulation and connect MATLAB to the same ROS domain.

Example Workflow
Terminal 1 (WSL)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=123
unset ROS_LOCALHOST_ONLY
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_fake_node turtlebot3_fake_node.launch.py

In MATLAB (Command Window)
setenv('ROS_DOMAIN_ID','123');
main_ros_*


Each main_ros_* file represents a different path:

main_ros_square.m → square path

main_ros_crcl.m → circular path

main_ros_sqiggly.m → curved “squiggly” path

main_ros_J.m → J-shaped path

 Results (What You’ll See)

Smooth trajectory tracking across custom waypoints

Continuous velocities at waypoint boundaries

Real-time plots of position, curvature, and wheel speeds

Animated visualization of TurtleBot3 motion

File Overview
File	Description
make_geom_from_waypoints.m	Smooths discrete waypoints using cubic splines
eval_path.m	Evaluates path positions, derivatives, and curvature
poly_traj_coeffs_vdes.m	Computes cubic polynomial coefficients for time scaling
fcn_traj_dd.m	Generates time-parameterized trajectory
fcn_controller_dd.m	Feedback controller for differential drive tracking
dyn_diffdrive.m	Kinematic model for offline simulation
main.m	MATLAB-only trajectory simulation
main_ros_*.m	ROS2-connected scripts for TurtleBot3 simulation
Extensibility

The modular design allows simple extensions:

To add Obstacle avoidance 

Add a separate function (for example plan_avoidance.m) that pre-processes the waypoint list. It checks each straight line segment between consecutive waypoints against known obstacles. If a segment is too close, it shifts affected waypoints outward by an offset and returns the modified list, which you then pass into this pipeline exactly like the original waypoints.
Rule:
Let D_maxbe the maximum robot dimension. Set the safety offset
Δ=1.5" " D_max.

For each segment (P_i P_(i+1) ) ‾, compute the minimum distance to each obstacle. If
"dist"((P_i P_(i+1) ) ‾,"obstacle")<Δ,

adjust the segment locally by moving the relevant waypoint(s) along the outward normal by Δ. The resulting modified waypoints are then fed into make_geom_from_waypoints.m with no other code changes.

