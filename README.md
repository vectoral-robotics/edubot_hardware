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
- `led_node.py` — corner status LEDs (NeoPixel/WS2812B over SPI)
- `led_interface.py` — LED backends (real SPI + null) and colour helpers

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

## Corner LEDs (NeoPixel / WS2812B)

One addressable RGB LED per corner, chained on a single data line and driven
from the Raspberry Pi 5 over **SPI** (not the ESP32). SPI is used because the
PIO path of `adafruit-circuitpython-neopixel` fails on the Pi 5 / Ubuntu 24.04
(`Failed to open PIO device`, no `/dev/pio0`).

**Wiring** (single data line for the whole chain):

| NeoPixel | Raspberry Pi 5 |
|---|---|
| DIN | GPIO10 / MOSI (pin 19) |
| 5V  | pin 2 |
| GND | pin 6 |

Chain: `MOSI → DIN(LED0)`, `DOUT(LED0) → DIN(LED1)`, … one line, N LEDs.

**Run it:**

```bash
# Real hardware (needs SPI enabled — see below)
ros2 run edubot_hardware led_node

# No hardware (dev laptop / sim): uses a logging null backend
ros2 run edubot_hardware led_node --ros-args -p use_sim:=true
```

The node also falls back to the null backend automatically if the SPI device or
the adafruit library is missing, so it never brings the stack down.

**Interfaces** — one topic per corner plus an `all` topic, each a plain
`std_msgs/ColorRGBA` with **r/g/b in 0–255** (not the 0–1 RViz convention).

| Topic | Type | Sets |
|---|---|---|
| `/led/front_left` | `std_msgs/ColorRGBA` | the front-left corner |
| `/led/front_right` | `std_msgs/ColorRGBA` | the front-right corner |
| `/led/rear_left` | `std_msgs/ColorRGBA` | the rear-left corner |
| `/led/rear_right` | `std_msgs/ColorRGBA` | the rear-right corner |
| `/led/all` | `std_msgs/ColorRGBA` | all corners at once |

Values are clamped to 0–255, so an out-of-range publish can't crash the node.

```bash
# One corner red
ros2 topic pub -1 /led/front_left std_msgs/ColorRGBA "{r: 255, g: 0, b: 0}"

# All corners white
ros2 topic pub -1 /led/all std_msgs/ColorRGBA "{r: 255, g: 255, b: 255}"
```

**Boot animation → ready.** On start-up all four corners gently *breathe* in a
neutral cool white. When the robot becomes usable — the fleet `run.sh` waits for
the dashboard port, then latches `/robot_ready` (`std_msgs/Bool`, `true`) — the
LEDs smoothly fade to a **steady** glow. Publishing `false` returns them to
breathing. The first manual `/led/...` command takes over and stops the
animation entirely. To drive readiness yourself:

```bash
ros2 topic pub -1 /robot_ready std_msgs/Bool "{data: true}" \
  --qos-durability transient_local
```

**Key parameters:** `use_sim`, `num_pixels` (4), `brightness` (0.4),
`pixel_order` (`GRB`), `corner_names` (topic suffix per corner),
`startup_color` (`[r, g, b]` 0–255), `clear_on_shutdown`. Boot animation:
`boot_animation` (true), `breath_color` (`[200, 225, 255]` cool white),
`breath_period` (4.0 s), `breath_min` (0.1), `animation_hz` (30),
`ready_fade` (1.2 s). Corner index follows the physical chain
(pixel 0 = first LED after MOSI).

**Enable SPI (once per robot).** `/dev/spidev0.0` must exist. Use the helper in
the meta-repo, then reboot:

```bash
make enable-spi     # adds 'dtparam=spi=on' to /boot/firmware/config.txt
sudo reboot
ls /dev/spidev0.0   # verify
```

In the fleet stack the `edubot` container runs privileged with `/dev` mapped in,
so it accesses `/dev/spidev0.0` directly — no extra device flags needed. The
Python dependency (`adafruit-circuitpython-neopixel-spi`) is baked into the ROS
image. Enable/disable the node per robot via `ENABLE_LEDS` (default `true`).

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
