# Robot Simulation with SLAM and Navigation (Gazebo + RViz3)

This package provides a simulated mobile robot that performs **Simultaneous Localization and Mapping (SLAM)** in Gazebo and visualizes data in **RViz3**. After creating a map, you can switch to **navigation mode** using Nav2.

---

## Features
- Full robot simulation in Gazebo.
- Live visualization in RViz3.
- SLAM capability to generate maps of the environment.
- Nav2 navigation stack for autonomous movement once a map is built.
- Keyboard teleoperation support.


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
