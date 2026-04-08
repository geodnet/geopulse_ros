# Geodnet ROS2 Geopulse Driver

A ROS2 driver for parsing NMEA sentences from Geodnet GNSS/GPS devices, publishing standard ROS navigation messages with extended support for Quectel IMU data. Does not require the GPSD daemon.

## Table of Contents

- [Device Requirements](#device-requirements)
- [Features](#features)
- [Published Topics](#published-topics)
- [Windows / WSL Setup](#windows--wsl-setup)
- [Quick Start — Docker](#quick-start--docker)
- [Quick Start — Native](#quick-start--native)
- [Configuration](#configuration)
- [RViz Visualization](#rviz-visualization)
- [Troubleshooting](#troubleshooting)
- [Recording & Playback](#recording--playback)
- [Attribution](#attribution)

---

## Device Requirements

This driver requires the GEOPULSE device to be running **firmware version 3.6.0 or later**. Older firmware versions may not output the required NMEA sentences.

### Firmware Upgrade Procedure

1. Plug the GEOPULSE device into your PC or laptop via USB.
2. Open a serial terminal (e.g. RealTerm, PuTTY, or similar).
3. Configure WiFi on the device by sending:
   ```
   +HYFIX,WIFI,"wifiname","wifipassword"#
   ```
   Replace `wifiname` and `wifipassword` with your network credentials.
4. Send the upgrade command to start the firmware update.
5. Wait for the device to complete the upgrade and reboot.

After the upgrade completes, reconnect the device before continuing.

---



- Standard NMEA sentence parsing: `GGA`, `RMC`, `VTG`
- Proprietary Quectel sentence support:
  - `PQTMSENMSG` — Raw IMU data (accelerometer + gyroscope)
  - `PQTMDRPVA` — INS position, velocity, and attitude
- Serial port connection
- Publishes standard ROS2 message types (see [Published Topics](#published-topics))

---

## Published Topics

| Topic | Type | Description |
|---|---|---|
| `/fix` | `sensor_msgs/NavSatFix` | GPS fix with position and covariance |
| `/vel` | `geometry_msgs/TwistStamped` | Velocity from GPS/INS |
| `/time_reference` | `sensor_msgs/TimeReference` | GPS time reference |
| `/imu/data` | `sensor_msgs/Imu` | Fused IMU data with orientation |
| `/imu/data_raw` | `sensor_msgs/Imu` | Raw IMU data (accel + gyro only) |
| `/odometry/ins` | `nav_msgs/Odometry` | Full INS odometry |

---

## Windows / WSL Setup

If you are on Windows, we recommend running the driver inside Ubuntu via WSL2. This gives you the Linux environment that ROS2 requires while staying on Windows.

**Requirements:**
- Ubuntu via WSL2
- Docker Desktop with WSL integration enabled

**Before starting:** make sure the GEOPULSE USB device is visible inside WSL. You can verify this by checking that `/dev/ttyUSB0` (or similar) appears after plugging in the device.

### Accessing ROS Topics Inside Docker (WSL)

When running via Docker, ROS2 commands must be run inside the container. From your WSL terminal:

```bash
cd ~/ros2_ws/src/geopulse_ros
docker compose exec nmea-driver bash
```

Then inside the container:

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
ros2 topic list
```

---

## Quick Start — Docker

### 1. Clone the repo into a ROS2 workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <this-repo-url>
cd ~/ros2_ws
```

### 2. Build the Docker image

```bash
./build.sh
```

To target a specific ROS distro:

```bash
ROS_DISTRO=jazzy ./build.sh
```

### 3. Run

```bash
docker compose up
```

If your device is not at `/dev/ttyUSB0`, override it:

```bash
SERIAL_DEVICE=/dev/ttyACM0 docker compose up
```

---

## Quick Start — Native

### Prerequisites

- ROS2 Humble or later
- Python 3
- ROS2 packages: `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `nmea_msgs`, `rclpy`, `tf_transformations`

### 1. Clone into your ROS2 workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <this-repo-url>
```

### 2. Install dependencies

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build and source

```bash
colcon build --packages-select nmea_navsat_driver
source install/setup.bash
```

### 4. Launch

```bash
ros2 launch nmea_navsat_driver nmea_serial_driver.launch.py
```

---

## Configuration

Edit `config/nmea_serial_driver.yaml` to match your device:

```yaml
nmea_navsat_driver:
  ros__parameters:
    port: "/dev/ttyUSB0"
    baud: 115200
    frame_id: "gps"
    time_ref_source: "gps"
    useRMC: False
```

---

## RViz Visualization

Visualize your device's real-time position and orientation. Requires a Quectel LC29H (or similar) with DR/INS enabled and outputting `PQTMDRPVA` messages.

### Step 1 — Start all nodes

You'll need 4 terminals. Open them all before starting RViz.

**Terminal 1 — NMEA driver:**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run nmea_navsat_driver nmea_serial_driver --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p baud:=115200 \
  -p frame_id:=base_link
```

**Terminal 2 — Odometry TF broadcaster:**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run nmea_navsat_driver odom_tf_broadcaster
```

**Terminal 3 — Static TF (map → odom):**
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```

**Terminal 4 — RViz:**
```bash
ros2 run rviz2 rviz2
```

### Step 2 — Configure RViz

Once RViz is open, configure it as follows:

**Fixed Frame**

- Top-left dropdown → change `map` to `odom`

**Add TF display**

- Click **Add** → select **TF** → click **OK**
- You should see: `map` → `odom` → `base_link`

**Add Axes display**

- Click **Add** → select **Axes** → click **OK**
- Set properties: Reference Frame: `base_link` | Length: `2.0` | Radius: `0.3`

**Add Grid display**

- Click **Add** → select **Grid** → click **OK**
- Set properties: Reference Frame: `odom` | Cell Count: `20` | Cell Size: `1`

**Add Odometry display (optional)**

- Click **Add** → select **Odometry** → click **OK**
- Set properties: Topic: `/odometry/ins` | Shape: `Arrow` | Arrow Length: `2.0`

**Camera view**

- Right panel → **Views** → Type: **Orbit**
- Target Frame: `base_link` | Distance: `10` | Yaw: `0` | Pitch: `0.785` (45°)

### Step 3 — Verify

In a new terminal, confirm everything is publishing:

```bash
# TF should show ~10 Hz
ros2 topic hz /tf

# Tilt the device — these values should change
ros2 topic echo /odometry/ins --field pose.pose.orientation
```

### Step 4 — Test movement

| Action | Expected result |
|---|---|
| Tilt forward | Green (Y) axis tilts forward, Red (X) tilts up |
| Tilt left | Red (X) axis tilts left, Blue (Z) rotates |
| Rotate (yaw) | Axes spin around Blue (Z) axis |
| Walk 10 m | Axes translate across the grid |

### Coordinate frames

```
map (static world frame)
 └─ odom (local ENU frame, origin = first GPS fix)
     └─ base_link (device frame, moves with sensor)
```

The grid uses East-North-Up (ENU) coordinates:
- **X (Red)** → East
- **Y (Green)** → North
- **Z (Blue)** → Up
- **Origin (0,0,0)** → your device's first GPS position

### Save and reload RViz config

```bash
# Save: File → Save Config As...
# Suggested path:
~/ros2_ws/src/nmea_navsat_driver/config/ins_visualization.rviz

# Reload later:
ros2 run rviz2 rviz2 -d ~/ros2_ws/src/nmea_navsat_driver/config/ins_visualization.rviz
```

---

## Troubleshooting

### No axes visible in RViz

Make sure all 4 terminals are running.

```bash
ros2 node list
# Expected: /nmea_serial_driver, /odom_tf_broadcaster, /static_tf_map_odom, /rviz2

ros2 topic list | grep tf
# Expected: /tf, /tf_static
```

### Axes don't move

```bash
ros2 topic hz /odometry/ins         # Should show ~10 Hz
ros2 topic echo /fix --field status # Check for GPS lock
```

Ensure the device has GPS lock and INS/DR mode is enabled and calibrated.

### TF errors in RViz

```bash
ros2 run tf2_tools view_frames      # Opens frames.pdf showing TF tree
ros2 run tf2_ros tf2_echo odom base_link  # Should show a live transform
```

Make sure `odom_tf_broadcaster` is running and `/odometry/ins` is publishing.

### Axes orientation seems wrong

The driver converts the device's clockwise-from-North heading to the ROS ENU convention (counter-clockwise from East). To verify:

- Face North → Red arrow should point North
- Face East → Red arrow should point East

If wrong, check the yaw conversion in `driver.py`.

### Position doesn't match GPS lat/lon

This is expected. The visualization uses **relative ENU coordinates** — not absolute GPS coordinates. The origin is your first GPS fix. Walking 10 m East should show position change from `(0,0,0)` to approximately `(10,0,0)`.

---

## Monitoring Performance

```bash
# Message rates
ros2 topic hz /odometry/ins
ros2 topic hz /tf
ros2 topic hz /imu/data

# Data quality
ros2 topic echo /fix --field status              # GPS fix status
ros2 topic echo /odometry/ins --field pose.covariance  # Position accuracy
```

---

## Recording & Playback

```bash
# Record all topics
ros2 bag record -a

# Or record specific topics
ros2 bag record /odometry/ins /imu/data /fix /tf /tf_static

# Playback
ros2 bag play <bag_file_name>
ros2 run rviz2 rviz2 -d ~/ros2_ws/src/nmea_navsat_driver/config/ins_visualization.rviz
```

---

## Tips for Best Results

1. **Get good GPS signal** — stand outside with a clear sky view
2. **Calibrate INS before use** — walk in a figure-8 pattern and tilt the device in various orientations
3. **Keep device level at startup** — the first position becomes the origin
4. **Wait for GPS lock** — solution type should be ≥ 1; RTK Fixed (type 4) is best
5. **Move smoothly** — sudden jerky movements can cause temporary errors

---

## Additional RViz Displays

**Path (trajectory trail):**
- Add → **Path** | Topic: `/path` | Color: Green | Line Width: `0.05`
- Requires a separate path publisher node

**IMU visualization:**
```bash
sudo apt install ros-${ROS_DISTRO}-rviz-imu-plugin
# In RViz: Add → rviz_imu_plugin → Imu | Topic: /imu/data
```

---

## Attribution

Based on the [nmea_navsat_driver](https://github.com/ros-drivers/nmea_navsat_driver) ROS2 package.
Copyright (c) 2013, Eric Perko. Licensed under the BSD License.

**Links:**
- [ROS Wiki](http://ros.org/wiki/nmea_navsat_driver)
- [Original GitHub](https://github.com/ros-drivers/nmea_navsat_driver)
- [ROS2 TF2 docs](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)
- [nav_msgs/Odometry](https://docs.ros2.org/latest/api/nav_msgs/msg/Odometry.html)
- [sensor_msgs/Imu](https://docs.ros2.org/latest/api/sensor_msgs/msg/Imu.html)