## v0.3.0 (2026-07-20)

### Feat

- **imu**: add BNO085 IMU node publishing sensor_msgs/Imu on imu/data

## v0.2.0 (2026-07-15)

### Feat

- **leds**: add corner NeoPixel LED node over SPI

### Fix

- **led**: use relative topic names so led_node honours its namespace

### Refactor

- **led**: NaN-safe clamp8, shared backend logging, drop dead helpers
- **leds**: move boot animation to host systemd, simplify led_node
- **leds**: move boot animation to host systemd, simplify led_node

## v0.1.1 (2026-07-02)

### Fix

- **ci**: push annotated tag so the version tag is published (#4)

## v0.1.0 (2026-07-02)

### Feat

- **odometry**: publish odometry covariances

### Fix

- **license**: align setup.py with PolyForm Perimeter 1.0.0
- ros hardware node - new esp32
- ros hardware node - new esp32
- hardware node seperate frequence for TF and odom
- encoder dt calculation on arduino
- wheel setup

### Refactor

- rename omnibot to edubot across the repo
- hardware node
