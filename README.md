# nav_bot-gazebo_harmonic

A ROS 2 robot simulation package featuring differential-drive navigation with Gazebo Harmonic integration. Includes robot description (URDF/SDF), launch orchestration, sensor simulation (LiDAR), and Nav2 navigation stack configuration.

## Features

- **Robot Model**: Differential-drive robot with LiDAR sensor
- **Simulation**: Gazebo Harmonic with physics and sensor plugins
- **Navigation**: Nav2 stack integration with EKF odometry fusion
- **Visualization**: RViz configuration for real-time monitoring
- **ROS 2**: Built on ROS 2 Humble with Python launch files

## Directory Structure

```
nav2_pkg/
├── src/nav_bot/                    # Main ROS 2 package
│   ├── nav_bot/                    # Python package module
│   │   ├── my_bot.urdf             # Robot URDF description
│   │   └── my_bot.sdf              # Gazebo SDF model
│   ├── launch/
│   │   └── display.launch.py        # Main launch file
│   ├── config/
│   │   ├── nav2_params.yaml         # Nav2 navigation parameters
│   │   └── ekf.yaml                 # EKF odometry fusion config
│   ├── rviz/
│   │   └── config.rviz              # RViz visualization config
│   ├── world/
│   │   └── my_world.sdf             # Gazebo world file
│   ├── package.xml                  # ROS 2 package metadata
│   ├── setup.py                     # Python package setup
│   └── test/                        # Unit tests (copyright, flake8, pep257)
├── lidar.urdf                       # LiDAR sensor URDF
└── README.md                        # This file
```

## Prerequisites

- **ROS 2**: Humble or later
- **Gazebo**: Gazebo Harmonic
- **Python**: 3.10+
- **Dependencies**: 
  - `gazebo_ros`
  - `robot_state_publisher`
  - `rviz2`
  - `nav2_bringup`
  - `robot_localization` (for EKF)

## Installation

1. **Clone the repository:**
   ```bash
   git clone https://github.com/rohan2104jadhav/nav_bot-gazebo_harmonic.git
   cd nav2_pkg
   ```

2. **Install dependencies:**
   ```bash
   rosdep install --from-paths src --ignore-src -y
   ```

3. **Build the package:**
   ```bash
   colcon build --packages-select nav_bot
   source install/setup.bash
   ```

## Usage

### Launch the Simulation

Start the robot simulation with visualization and navigation stack:

```bash
ros2 launch nav_bot display.launch.py
```

This will start:
- Gazebo simulator with robot and world
- Robot State Publisher (TF tree broadcasting)
- RViz visualization
- EKF odometry filter
- Nav2 navigation stack (if configured)

### Robot Topics

**Publisher Topics:**
- `/odom` — Odometry from robot motion
- `/tf` — Transform tree (robot links and joints)
- `/robot_description` — URDF/SDF robot model
- `/scan` — LiDAR sensor data (if LiDAR is active)

**Subscriber Topics:**
- `/cmd_vel` — Velocity commands (geometry_msgs/Twist)
  ```bash
  ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}, angular: {z: 0.2}}"
  ```

### Configuration

- **Navigation Parameters**: Edit `src/nav_bot/config/nav2_params.yaml` to adjust planner behavior, controller gains, etc.
- **EKF Configuration**: Modify `src/nav_bot/config/ekf.yaml` to change odometry fusion settings
- **Robot Model**: Update `src/nav_bot/nav_bot/my_bot.urdf` for geometric changes or `my_bot.sdf` for Gazebo-specific physics
- **World**: Customize `src/nav_bot/world/my_world.sdf` to add obstacles, lighting, or other objects

## Architecture

### Data Flows

1. **Robot Description** → URDF/SDF parsed → published to `/robot_description`
2. **Gazebo Simulation** → spawns robot model → publishes `/odom`, `/tf`
3. **EKF Fusion** → consumes `/odom` and IMU data → produces filtered odometry
4. **Navigation Stack** → subscribes to odometry and `/scan` → publishes `/cmd_vel` commands
5. **Visualization** → RViz consumes `/tf` and sensor data for real-time display

### Key Components

| Component | Purpose | Configuration |
|-----------|---------|---------------|
| Gazebo Harmonic | Physics simulation & sensor plugins | Implicit in SDF files |
| Robot State Publisher | Publishes TF tree | Reads URDF from `/robot_description` |
| EKF Localization Filter | Fuses odometry sources | `ekf.yaml` |
| Nav2 Stack | Path planning & navigation | `nav2_params.yaml` |
| RViz | Visualization | `rviz/config.rviz` |

## Troubleshooting

### Robot not appearing in Gazebo
- Ensure `gazebo_ros` package is installed: `sudo apt install ros-humble-gazebo-ros*`
- Check URDF/SDF syntax: `check_urdf src/nav_bot/nav_bot/my_bot.urdf`
- Verify model spawn: `ros2 service call /spawn_entity gazebo_msgs/SpawnEntity ...`

### No TF tree
- Confirm Robot State Publisher is running: `ros2 node list | grep state_publisher`
- Check `/robot_description` topic: `ros2 topic echo /robot_description | head`

### Navigation stack not responding
- Verify Nav2 is enabled in launch file
- Check parameters: `ros2 param list | grep nav2`
- Review logs: `ros2 launch nav_bot display.launch.py --log-level debug`

### LiDAR not publishing `/scan`
- Ensure LiDAR plugin is correctly configured in `my_bot.sdf`
- Verify Gazebo is running: `ps aux | grep gzserver`

## Development

### Running Tests

```bash
colcon test --packages-select nav_bot
colcon test-result --all --verbose
```

### Code Style

The package uses:
- **flake8** for Python linting
- **pep257** for docstring conventions
- **copyright** checker for license headers

Run linters:
```bash
python3 -m flake8 src/nav_bot/nav_bot/ src/nav_bot/setup.py
python3 -m pep257 src/nav_bot/nav_bot/ src/nav_bot/setup.py
```

## Contributing

Contributions are welcome! Please:
1. Fork the repository
2. Create a feature branch: `git checkout -b feature/your-feature`
3. Commit changes: `git commit -am "Add your feature"`
4. Push to branch: `git push origin feature/your-feature`
5. Open a Pull Request

## License

See `src/nav_bot/LICENSE` for details.

## Maintainer

**Rohan** — rohan2104jadhav@example.com

## References

- [ROS 2 Documentation](https://docs.ros.org/)
- [Gazebo Harmonic Docs](https://gazebosim.org/docs/harmonic/)
- [Nav2 Documentation](https://docs.nav2.org/)
- [URDF Specification](http://wiki.ros.org/urdf)

## Quick Commands Reference

```bash
# Build
colcon build --packages-select nav_bot

# Source setup
source install/setup.bash

# Launch
ros2 launch nav_bot display.launch.py

# Check topics
ros2 topic list
ros2 topic echo /odom

# Send commands
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 1.0}}"

# View TF tree
ros2 run rqt_tf_tree rqt_tf_tree

# RViz visualization
rviz2 -d src/nav_bot/rviz/config.rviz
```

---

**Last Updated**: November 27, 2025  
**ROS 2 Version**: Humble  
**Gazebo Version**: Harmonic
