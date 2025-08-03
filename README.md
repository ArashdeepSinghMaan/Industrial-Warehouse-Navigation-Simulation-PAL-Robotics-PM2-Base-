# Industrial-Warehouse-Navigation-Simulation-PAL-Robotics-PM2-Base-
Industrial Warehouse Navigation Simulation (PAL Robotics PM2 Base)
# Industrial Warehouse Simulation (PAL Robotics PM2 Base)

This project simulates **industrial warehouse automation** using the **PAL Robotics PM2 base** in **Gazebo**, powered by **ROS 2 Humble**.  
It demonstrates multi-zone navigation, advanced mapping, stable localization, and modular Behavior Trees for warehouse operations like **pick–place–recharge**.

---

## Features
- **Multi-Zone Navigation**
  - Path planning & control using **Nav2** with the **TEB Planner & Controller**
- **Mapping & Localization**
  - **RTAB-Map** for mapping (RGB-D or 3D lidar-based)
  - **AMCL** for localization on saved maps
  - Odometry and IMU fusion using **robot_localization**
- **Behavior Tree Workflow**
  - Modular tasks (pick, place, recharge) implemented using **BehaviorTree.CPP v3**
- **Simulation**
  - PAL Robotics PM2 base simulation in **Gazebo**
  - Warehouse world with multiple operational zones

---

## Tools & Technologies
- ROS 2 Humble
- Nav2 (TEB Planner & Controller)
- RTAB-Map
- AMCL
- BehaviorTree.CPP v3
- robot_localization
- Gazebo
- C++

---

## Project Structure
