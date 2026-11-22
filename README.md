# ROS 2 Mobile Robot Sensor Fusion in Gazebo

A ROS 2 and Gazebo simulation workspace for local occupancy-grid fusion using a 2D LiDAR and three short-range directional proximity sensors.

## Project overview

The `sensor_fusion_bot` package launches a differential-drive robot in a Gazebo test world. A sensor-fusion node consumes the LiDAR and proximity-sensor topics, uses TF2 to account for sensor poses, performs ray-based occupancy updates with Bayesian log-odds, and publishes a fused local occupancy grid and a derived scan.


<img width="1366" height="768" alt="Screenshot from 2026-11-22 12-12-30" src="https://github.com/user-attachments/assets/d60eb07f-1d32-4b6f-b018-e8e52aebfa77" />
<img width="1366" height="768" alt="Screenshot from 2026-11-22 12-16-40" src="https://github.com/user-attachments/assets/cb48d9e3-c372-446f-b718-d4b154dda6a8" />


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

