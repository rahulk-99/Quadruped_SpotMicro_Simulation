# ROS2 Quadruped Robot Project
https://github.com/user-attachments/assets/2d3996f6-25c4-4fd8-bb20-26cde8555218
> Quadruped robot simulation in Gazebo
This repository contains the code for a quadruped robot(spotMicro) in ROS2. It provides a simulation environment using Gazebo and visualization in RViz, along with a controller node for robot operation.

## Prerequisites

Before proceeding, make sure you have the following installed:

- **ROS2** (preferably the latest stable version, e.g., ROS 2 Humble)
- **colcon** build tool
- **git** version control system
- **Gazebo** (for robot simulation)
- **rviz** (for visualization of robot data)
- **pybullet** (pip install pybullet for pybullet visualisation)

## Installation Instructions

Follow these steps to set up the project on your local machine:

### 1. Create a Folder for the Project

Choose a location on your system to store the project. For example, create a new directory:

```bash
mkdir ros2_quadruped_project
```

### 2.Navigate to the folder

```bash
cd ros2_quadruped_project
```
### 3.Clone the Repository

```bash
git clone https://github.com/rahulk-99/Quadruped_SpotMicro_Simulation.git
```
### 4.Go inside the project directory



### 5.Build the package

```bash
colcon build
```


### 6.Source the setup file

```bash
source install/setup.bash
```

## Running the Project


### 7.Launch the empty world in Gazebo
To start the simulation with an empty world, use the following command:
```bash
ros2 launch quadruped_robot gazebo.launch.py
```

### 8.Run the Controller Node
Open another terminal, source the setup file again, and run the controller node with the following command:
```bash
source install/setup.bash
ros2 run quadruped_controller controller_node
```
### 9.Visualize Data in RViz
To visualize the robot's data in RViz, run the following command in a new terminal:
```bash
ros2 launch quadruped_robot display.launch.py
```
### 10.Visualize robot in Pybullet
To visualize the robot in Pybullet, run the following file in python installing Pybullet library with pip install Pybullet
```bash
Pybullet.ipynb
```
## Troubleshooting
If you encounter errors about missing dependencies, you can install them using rosdep:
```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## Spotmicro Repository

The models used can be found here: https://www.thingiverse.com/thing:3445283





