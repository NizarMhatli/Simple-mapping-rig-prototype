# Simple Mapping Rig

ROS 2 Jazzy Docker-based simulation of a sensor jig with Livox MID360 LiDAR and built-in IMU.

## Objective

Instrumentation and mapping using LiDAR is a growing field with highly sophisticated commercial solutions:
- [3D Scantech](https://www.3d-scantech.com/product_category/color-3d-scanner/)
- [RIEGL](https://www.unmannedsystemstechnology.com/company/riegl/)

The goal of this project is to explore whether a **budget-friendly scanner jig** can be built and used across different applications — from construction site mapping to industrial inspection.

## TODO
- [ ] Add a GUI to record and manage scan data
- [ ] Add localization and map creation features (SLAM)
- [ ] Test with real Livox MID360 + Jetson Orin Nano hardware
- [ ] Replace placeholder STL meshes with real CAD files

## Jig Model
![Jig Model](media/demo.png)

## Demo

![RViz](media/rviz.gif)
![Gazebo](media/gaz.gif)

## Hardware
- **Base:** Jetson Orin Nano
- **LiDAR:** Livox MID360 (with built-in IMU)

## Quick Start

```bash
git clone https://github.com/NizarMhatli/simple-mapping-jig.git
cd simple-mapping-jig
mkdir -p build install log maps config
touch config/.bash_history
make build
make up
make shell
```

Inside container:
```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch jig_description gazebo.launch.py
```

## Package Structure
```
workspace/
└── jig_description/
    ├── urdf/          # Xacro URDF files
    ├── meshes/        # STL files
    ├── launch/        # Launch files
    ├── config/        # Gazebo bridge config
    ├── worlds/        # Gazebo world with obstacles
    └── rviz/          # RViz config
```
