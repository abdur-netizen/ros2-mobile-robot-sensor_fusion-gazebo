# ROS 2 Mobile Robot Sensor Fusion in Gazebo

A ROS 2 and Gazebo simulation workspace for local occupancy-grid fusion using a 2D LiDAR and three short-range directional proximity sensors.

## Project overview

The `sensor_fusion_bot` package launches a differential-drive robot in a Gazebo test world. A sensor-fusion node consumes the LiDAR and proximity-sensor topics, uses TF2 to account for sensor poses, performs ray-based occupancy updates with Bayesian log-odds, and publishes a fused local occupancy grid and a derived scan.

### Main components

- `urdf/fusion_bot.urdf` — robot model and simulated sensors.
- `worlds/fusion_test.sdf` — Gazebo test environment containing obstacles at different heights/positions.
- `config/bridge.yaml` — Gazebo/ROS topic bridge configuration.
- `launch/fusion_sim.launch.py` — simulation, robot description, bridge, and fusion-node launch orchestration.
- `sensor_fusion_bot/fusion_node.py` — sensor subscriptions, TF transforms, ray tracing, occupancy evidence updates, and map/scan publication.

## Workspace layout

```text
fusion_ws/
└── src/
    └── sensor_fusion_bot/
        ├── config/
        ├── launch/
        ├── resource/
        ├── sensor_fusion_bot/
        ├── test/
        ├── urdf/
        ├── worlds/
        ├── package.xml
        ├── setup.cfg
        └── setup.py
```

## Build and run

Run these commands in a ROS 2 environment with the ROS-Gazebo integration packages and dependencies used by this workspace installed:

```bash
cd ~/fusion_ws
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch sensor_fusion_bot fusion_sim.launch.py
```

The exact ROS 2 distribution and Gazebo release should match the environment in which the workspace was tested.

## Outputs

The fusion node is configured to publish:

- `/fused_local_map` — `nav_msgs/msg/OccupancyGrid`
- `/fused_scan` — `sensor_msgs/msg/LaserScan`

Sensor inputs include `/lidar/scan` and the center/left/right proximity topics configured by the package.

## Scope and current status

This project demonstrates simulated multi-sensor occupancy-grid fusion. It is not, by itself, a complete SLAM system or a Nav2 autonomous-navigation/obstacle-avoidance stack.

**Runtime status:** The project owner reports that the Gazebo runtime works without faults. This repository upload does not independently re-run that simulation. The implementation is being published as supplied; follow-up improvements should be validated against the working runtime before changing sensor or bridge behavior.

## Notes for further development

Potential next steps include documenting the exact tested ROS 2/Gazebo versions and launch procedure, adding a reproducible RViz configuration, unit tests for ray tracing and occupancy updates, and evaluating map-frame behavior during robot motion. Verify Gazebo-to-ROS message types against the actual running topics before modifying bridge configuration.
