## v0.6.0 (2026-07-30)

### Feat

- **hardware**: add publish_tf parameter to disable TF when EKF is active
- **speaker**: add generate_phrases console script
- **speaker**: instant pre-recorded phrases with Piper fallback

### Fix

- **test**: patch shutil.which in piper binary test and fix ruff formatting
- **speaker**: stronger edge smoothing and longer playback tail
- **speaker**: add configurable tail fade and silence smoothing
- **speaker**: smooth audio tail and keep volume control
- **speaker**: switch to en_US-lessac-high voice; pad silence to fix end click
- **speaker**: install phrases.json as package_data so node finds it at runtime
- **speaker**: remove duplicate appended code, clean up imports
- **speaker**: auto-resolve piper voice model when parameter is unset
- **speaker**: enforce parameter services and log active backend
- **speaker**: avoid blocking ROS callbacks during TTS playback
- **speaker**: default ALSA device to plughw:0
- **speaker**: fallback to default ALSA device on aplay failure
- **speaker**: subscribe to absolute speaker topics

## v0.5.0 (2026-07-26)

### Feat

- **speaker**: replace espeak-ng with Piper neural TTS

## v0.4.0 (2026-07-24)

### Feat

- **speaker**: add speaker bridge node with text/volume topics

### Fix

- **speaker**: play synthesized speech via aplay on the I2S sound card

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
