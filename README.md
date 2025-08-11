# Industrial-Warehouse-Navigation-Simulation-PAL-Robotics-PM2-Base-
Industrial Warehouse Navigation Simulation (PAL Robotics PM2 Base)
# Industrial Warehouse Simulation (PAL Robotics PM2 Base)

This project simulates **industrial warehouse automation** using the **PAL Robotics PM2 base** in **Gazebo**, powered by **ROS 2 Humble**.  
It demonstrates  navigation, advanced mapping, stable localization, and modular Behavior Trees for warehouse operations like **pick–place–recharge**.

---

## Features
- **Multi-Zone Navigation**
  - Path planning & control using **Nav2** with the **SMAC Planner & DWB Controller**
- **Mapping & Localization**
  - **SLAM TOOLBOX** for mapping ( 2D lidar-based)
  - **AMCL** for localization on saved maps
  - Odometry and IMU fusion using **robot_localization**
- **Behavior Tree Workflow**
  - Modular tasks will be implemented using **BehaviorTree.CPP v3**
- **Simulation**
  - PAL Robotics PM2 base simulation in **Gazebo**
  - Warehouse world with multiple operational zones

---

## Tools & Technologies
- ROS 2 Humble
- Nav2 (SMAC Planner & DWB Controller)
- RTAB-Map
- AMCL
- BehaviorTree.CPP v3
- robot_localization
- Gazebo
- C++

---

## Project Structure
