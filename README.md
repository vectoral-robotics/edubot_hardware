# edubot_hardware

Hardware interface and simulation backend for the [EduBot](https://github.com/vectoral-robotics) robot — by Vectoral.

## What it is

`edubot_hardware` is the bridge between ROS 2 and the robot's ESP32-based motor
controller. It converts body velocity commands (`/cmd_vel`) into mecanum wheel
speeds, streams them over serial to the ESP32, reads encoder feedback, and
integrates it into odometry. A drop-in simulation backend reproduces the same
protocol so the whole stack runs without hardware.

The package is intentionally split into small, testable pieces:

- `hardware_node.py` — the ROS 2 node (parameters, topics, timers)
- `mecanum_kinematics.py` — forward/inverse mecanum kinematics
- `odometry.py` — encoder-tick → pose/velocity integration
- `serial_interface.py` — serial bridge to the ESP32
- `simulation_interface.py` — protocol-compatible simulator

## Installation

Requires ROS 2 Humble.

```bash
cd ~/ros2_ws/src
git clone https://github.com/vectoral-robotics/edubot_hardware.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y   # installs python3-serial
colcon build --packages-select edubot_hardware
source install/setup.bash
```

## Usage

Normally this node is started by [`edubot_bringup`](https://github.com/vectoral-robotics/edubot_bringup).
To run it directly:

```bash
# Against real hardware
ros2 run edubot_hardware hardware_node --ros-args -p port:=/dev/ttyACM0

# In simulation (no serial device)
ros2 run edubot_hardware hardware_node --ros-args -p use_sim:=true
```

**Interfaces**

| Direction | Topic | Type |
|---|---|---|
| Subscribe | `/cmd_vel` | `geometry_msgs/Twist` |
| Publish | `/odom` | `nav_msgs/Odometry` |
| Publish | `/joint_states` | `sensor_msgs/JointState` |
| Publish (TF) | `odom → base_link` | — |

**Key parameters:** `use_sim`, `port`, `baud`, `wheel_radius`, `base_length`,
`base_width`, `ticks_per_rev`, `cmd_timeout`, `mecanum_layout` (`X`/`O`),
`odom_hz`, `tf_hz`. See `hardware_node.py` for defaults.

## Contributing

- Work on a short-lived feature branch and open a pull request against `main`
  (which is protected); changes land via PR review.
- Commit messages follow [Conventional Commits](https://www.conventionalcommits.org)
  (`feat:`, `fix:`, `docs:`, …). See `CLAUDE.md` for repo conventions.
- This is a Python package with linting/formatting via **ruff**. Install the
  git hooks once after cloning so checks run on every commit:

  ```bash
  pip install pre-commit && pre-commit install
  ```

## License

PolyForm Perimeter 1.0.0 (source-available) — see [LICENSE](LICENSE).
