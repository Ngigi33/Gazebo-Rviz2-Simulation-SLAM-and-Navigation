# Robot Simulation with SLAM and Navigation (Gazebo + RViz3)

This package provides a simulated mobile robot that performs **Simultaneous Localization and Mapping (SLAM)** in Gazebo and visualizes data in **RViz3**. After creating a map, you can switch to **navigation mode** using Nav2.

---

## Features
- Full robot simulation in Gazebo.
- Live visualization in RViz2.
- SLAM capability to generate maps of the environment.
- Nav2 navigation stack for autonomous movement once a map is built.
- Keyboard teleoperation support.

---
Prerequisites

- ROS 2 Humble (or compatible distribution).
- Gazebo (Fortress or the version paired with your ROS 2 release).
- RViz2.
- Nav2 packages installed (nav2_bringup).
- teleop_twist_keyboard package for keyboard teleoperation.
---

## System Overview

The diagram below shows how the components interact:

```mermaid
flowchart LR
    A[Gazebo Simulation] -->|Sensor data (Laser, Odometry)| B[SLAM Node]
    B -->|Map + Localization| C[RViz3 Visualization]
    C -->|User sets goals| D[Nav2 Navigation Stack]
    D -->|Velocity Commands| A
    E[Teleop Keyboard] -->|Manual Control| A
```
