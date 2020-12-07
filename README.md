
# Multi-robot Trajectory Planner

This repository contains the code for the paper:

Efficient Trajectory Planning for Multiple Non-holonomic Mobile Robots via Prioritized Trajectory Optimization

**Authors:** Juncheng Li, Maopeng Ran, and Lihua Xie from Nanyang Technological University.

This paper proposes an efficient trajectory planning approach that generates safe, dynamically feasible and near-optimal trajectories
for multiple non-holonomic mobile robots in obstacle-rich environments.

## 1. Software Requirements
* Ubuntu 16.04
* ROS Kinetic
* Octomap
* Ipopt

## 2. Installation instructions
#### (1) Install ROS Kinetic for Ubuntu 16.04
[ROS Installation](http://wiki.ros.org/ROS/Installation)

#### (2) Install Ipopt solver
[Ipopt Installation](https://coin-or.github.io/Ipopt/INSTALL.html)

#### (3) Install dependencies
```
sudo apt-get install ros-kinetic-octomap*
sudo apt-get install ros-kinetic-dynamic-edt-3d
sudo apt-get install cppad
```
#### (4) Build:
```
cd ~/catkin_ws/src
git clone https://github.com/LIJUNCHENG001/multi_robot_traj_planner.git
cd ../ && catkin_make
source ~/catkin_ws/devel/setup.bash
```
## 2. Run Simulations
There are two simulation environments available.
#### Warehouse
```
roslaunch multi_robot_traj_planner prioritized_plan_warehouse.launch 
```

<img width="35%" height="35%" src="multi_robot_traj_planner/img/warehouse.gif"/>

#### Environment with random obstacles
```
roslaunch multi_robot_traj_planner prioritized_plan_random_env.launch
```
## 3. Simulation Configuration


## 4. Acknowledgements

