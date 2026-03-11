# Geodnet ROS2 Geopulse Driver

A ROS2 driver for parsing NMEA sentences from Geodnet GNSS/GPS devices and publishing standard ROS navigation messages. This package is specifically enhanced for Geodnet devices with integrated IMU support.

## Overview

This driver parses NMEA strings from GPS/GNSS devices and publishes ROS2 messages without requiring the GPSD daemon. It includes extended support for proprietary Quectel NMEA sentences used by Geodnet devices.

## Features

- Standard NMEA sentence parsing (GGA, RMC, VTG)
- Proprietary Quectel sentence support:
  - `PQTMSENMSG` - Raw IMU data (accelerometer + gyroscope)
  - `PQTMDRPVA` - INS position, velocity, and attitude
- Serial port connection
- Publishes standard ROS2 message types:
  - `sensor_msgs/NavSatFix` - GPS fix data
  - `sensor_msgs/Imu` - IMU data (raw and fused)
  - `geometry_msgs/TwistStamped` - Velocity
  - `nav_msgs/Odometry` - Full odometry from INS

## Quick Start (Docker)

The easiest way to run the driver is with Docker.

### Build
```bash
./build.sh
```

Or specify a ROS distro:
```bash
ROS_DISTRO=jazzy ./build.sh
```

### Run
```bash
docker compose up
```

Override the serial device if needed:
```bash
SERIAL_DEVICE=/dev/ttyACM0 docker compose up
```

## Native Installation

### Prerequisites

- ROS2 (Humble or later recommended)
- Python 3
- Required ROS2 packages:
  - `geometry_msgs`
  - `sensor_msgs`
  - `nav_msgs`
  - `nmea_msgs`
  - `rclpy`
  - `tf_transformations`

### Build
```bash
# Clone into your ROS2 workspace
cd ~/ros2_ws/src
git clone <this-repo-url>

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --packages-select nmea_navsat_driver
source install/setup.bash
```

### Run
```bash
ros2 launch nmea_navsat_driver nmea_serial_driver.launch.py
```

## Configuration

Edit `config/nmea_serial_driver.yaml`:
```yaml
nmea_navsat_driver:
  ros__parameters:
    port: "/dev/ttyUSB0"
    baud: 115200
    frame_id: "gps"
    time_ref_source: "gps"
    useRMC: False
```

## Published Topics

| Topic             | Message Type                  | Description                        |
| ----------------- | ----------------------------- | ---------------------------------- |
| `/fix`            | `sensor_msgs/NavSatFix`       | GPS fix with position and covariance |
| `/vel`            | `geometry_msgs/TwistStamped`  | Velocity from GPS/INS              |
| `/time_reference` | `sensor_msgs/TimeReference`   | GPS time reference                 |
| `/imu/data`       | `sensor_msgs/Imu`             | Fused IMU data with orientation    |
| `/imu/data_raw`   | `sensor_msgs/Imu`             | Raw IMU data (accel + gyro only)   |
| `/odometry/ins`   | `nav_msgs/Odometry`           | Full INS odometry                  |

## Supported NMEA Sentences

### Standard Sentences

- `GGA` - GPS Fix Data
- `RMC` - Recommended Minimum Navigation Information
- `VTG` - Track Made Good and Ground Speed

### Proprietary Sentences (Geodnet/Quectel)

- `PQTMSENMSG` - IMU sensor message (accelerometer, gyroscope, temperature)
- `PQTMDRPVA` - Dead reckoning position, velocity, and attitude

## RViz Visualization Guide

This guide shows you how to visualize your GNSS/INS device's position and orientation in real-time using RViz2.

### Prerequisites for Visualization

- ROS 2 (Humble or later)
- nmea_navsat_driver package installed and built
- GNSS/INS device connected via USB (e.g., Quectel LC29H with DR/INS)
- Device calibrated and outputting PQTMDRPVA messages with orientation data

### Step 1: Start All Nodes

Open 4 terminals and run these commands:

**Terminal 1 - NMEA Driver:**
```bash
cd ~/geodnet_ros
source install/setup.bash
ros2 run nmea_navsat_driver nmea_serial_driver --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p baud:=115200 \
  -p frame_id:=base_link
```

**Terminal 2 - Odometry TF Broadcaster:**
```bash
cd ~/geodnet_ros
source install/setup.bash
ros2 run nmea_navsat_driver odom_tf_broadcaster
```

**Terminal 3 - Static TF (map→odom):**
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```

**Terminal 4 - RViz:**
```bash
ros2 run rviz2 rviz2
```

### Step 2: Configure RViz

When RViz opens, configure the following displays:

#### 1. Set Fixed Frame

- Top left dropdown labeled **"Fixed Frame"**
- Change from `map` to **`odom`**

#### 2. Add TF Display

- Click **"Add"** button (bottom left)
- Select **"TF"** from the list
- Click **"OK"**
- You should see frame connections: `map` → `odom` → `base_link`

#### 3. Add Axes Display

- Click **"Add"** button
- Select **"Axes"** from the list
- Click **"OK"**
- In the left panel under **Axes** properties, set:
  - **Reference Frame**: `base_link`
  - **Length**: `2.0`
  - **Radius**: `0.3`

#### 4. Add Grid

- Click **"Add"** button
- Select **"Grid"** from the list
- Click **"OK"**
- In properties:
  - **Reference Frame**: `odom`
  - **Plane Cell Count**: `20`
  - **Cell Size**: `1`

#### 5. Add Odometry Display (Optional)

- Click **"Add"** button
- Select **"Odometry"** from the list
- Click **"OK"**
- In properties:
  - **Topic**: `/odometry/ins`
  - **Shape**: `Arrow`
  - **Arrow Length**: `2.0`
  - **Color**: Red or your preference

#### 6. Set Camera View

- Right panel → **Views** section
- **Type**: Select **"Orbit"**
- **Target Frame**: `base_link`
- **Distance**: `10`
- **Yaw**: `0`
- **Pitch**: `0.785` (45 degrees)

### Step 3: Verify Everything is Working

**Terminal 5 - Check TF Broadcast Rate:**
```bash
ros2 topic hz /tf
# Should show ~10 Hz
```

**Terminal 6 - Check Orientation Updates:**
```bash
ros2 topic echo /odometry/ins --field pose.pose.orientation
```

**Tilt your device** - the quaternion values should change in real-time!

### Step 4: Test Movement

With RViz visible and configured, test the following:

#### Tilt Device Forward (Pitch):

- **Expected**: Green (Y) axis tilts forward, Red (X) axis tilts up
- **What it shows**: Device pitch angle

#### Tilt Device Left (Roll):

- **Expected**: Red (X) axis tilts left, Blue (Z) axis rotates
- **What it shows**: Device roll angle

#### Rotate Device (Yaw/Heading):

- **Expected**: Axes spin around Blue (Z) axis
- **What it shows**: Device heading/compass direction

#### Walk 10 Meters:

- **Expected**: Axes translate across the grid
- **What it shows**: Device position in ENU coordinates

### Expected Visualization

You should see 3 colored arrows (axes) representing your device orientation:

- **Red arrow (X axis)** - Points forward (device front)
- **Green arrow (Y axis)** - Points left (device left side)
- **Blue arrow (Z axis)** - Points up (device top)

These axes should:

- ✅ Rotate in real-time as you tilt/spin the device
- ✅ Move across the grid as you walk
- ✅ Update smoothly at ~10 Hz
- ✅ Follow your exact device movements with minimal lag

### Coordinate Frames
```
map (static world frame)
 └─ odom (local navigation frame, ENU coordinates)
     └─ base_link (device/sensor frame)
```

- **map**: Global reference frame (static)
- **odom**: Odometry frame with origin at first GPS position
- **base_link**: Device frame that moves and rotates with your sensor

### Understanding the Grid

The grid represents the local East-North-Up (ENU) coordinate system:

- **X axis (Red)**: Points East
- **Y axis (Green)**: Points North
- **Z axis (Blue)**: Points Up
- **Origin (0,0,0)**: Your device's first GPS position

As you move:

- Moving **East** increases X coordinate
- Moving **North** increases Y coordinate
- Moving **Up** (altitude) increases Z coordinate

## Troubleshooting

### No axes visible in RViz

**Check:**
```bash
# Verify nodes are running
ros2 node list
# Should show: /nmea_serial_driver, /odom_tf_broadcaster, /static_tf_map_odom, /rviz2

# Verify TF is being published
ros2 topic list | grep tf
# Should show: /tf, /tf_static
```

**Solution:** Make sure all 4 terminals are running (driver, odom_tf_broadcaster, static_tf, rviz)

### Axes don't move when device moves

**Check:**
```bash
# Verify odometry is publishing
ros2 topic hz /odometry/ins
# Should show ~10 Hz

# Verify orientation changes
ros2 topic echo /odometry/ins --field pose.pose.orientation
# Tilt device - values should change
```

**Solution:**

- Ensure device has GPS lock (check `/fix` topic)
- Verify PQTMDRPVA messages have non-zero roll/pitch/heading
- Check that device INS/DR mode is enabled and calibrated

### TF errors in RViz

**Check:**
```bash
# View TF tree
ros2 run tf2_tools view_frames
# Opens frames.pdf showing frame hierarchy

# Check specific transform
ros2 run tf2_ros tf2_echo odom base_link
# Should show updating transform
```

**Solution:** Ensure `odom_tf_broadcaster` is running and `/odometry/ins` is publishing

### Axes orientation seems wrong

**Check heading convention:** Your device outputs heading as clockwise from North. The driver converts this to ROS ENU convention (counter-clockwise from East).

**Test:**

- Face North → Red arrow should point North
- Face East → Red arrow should point East
- Face South → Red arrow should point South

If axes are rotated incorrectly, check the yaw conversion in `driver.py`.

### Position doesn't match GPS location

**Note:** The visualization uses **relative coordinates** (ENU), not absolute GPS coordinates. The origin (0,0,0) is set at your device's first GPS position.

**This is normal:** If you walk 10 meters East, you should see position change from (0,0,0) to (~10,0,0), not your GPS lat/lon.

## Advanced Configuration

### Save RViz Configuration

Once you have RViz configured:

1. **File** → **Save Config As...**
2. Save to: `~/geodnet_ros/src/nmea_navsat_driver/config/ins_visualization.rviz`

### Load Saved Configuration
```bash
ros2 run rviz2 rviz2 -d ~/geodnet_ros/src/nmea_navsat_driver/config/ins_visualization.rviz
```

### Change Camera View

For different perspectives:

**Top-down view:**

- Views → Orbit
- Pitch: `1.57` (90 degrees)
- Yaw: `0`

**Follow device:**

- Views → ThirdPersonFollower
- Target Frame: `base_link`
- Distance: `5-10`

**Side view:**

- Views → Orbit
- Pitch: `0`
- Yaw: `1.57` (90 degrees)

## Monitoring Performance

### Check Message Rates
```bash
# Odometry rate
ros2 topic hz /odometry/ins

# TF rate
ros2 topic hz /tf

# IMU rate (if enabled)
ros2 topic hz /imu/data
```

### Check Data Quality
```bash
# GPS fix status
ros2 topic echo /fix --field status

# Position covariance (accuracy)
ros2 topic echo /odometry/ins --field pose.covariance

# Solution type (from PQTMDRPVA)
ros2 topic echo /odometry/ins --once | grep solution
```

## Tips for Best Results

1. **Ensure good GPS signal** - Stand outside with clear sky view
2. **Calibrate INS before use** - Walk in figure-8 pattern, tilt device in various orientations
3. **Keep device level** when setting origin - First position becomes (0,0,0)
4. **Move smoothly** - Sudden jerky movements may cause temporary errors
5. **Wait for GPS lock** - Solution type should be 1 or higher (4=RTK Fixed is best)

## Recording and Playback

### Record a session
```bash
# Record all topics
ros2 bag record -a

# Or record specific topics
ros2 bag record /odometry/ins /imu/data /fix /tf /tf_static
```

### Playback
```bash
# Play back recorded data
ros2 bag play <bag_file_name>

# Then open RViz to visualize
ros2 run rviz2 rviz2 -d ~/geodnet_ros/src/nmea_navsat_driver/config/ins_visualization.rviz
```

## Additional Displays

### Add Path Display

Shows the trajectory your device has traveled:

1. Click **"Add"** → **"Path"**
2. Topic: `/path` (requires path publisher node)
3. Color: Green
4. Line Width: `0.05`

### Add IMU Display (requires plugin)
```bash
# Install IMU plugin
sudo apt install ros-${ROS_DISTRO}-rviz-imu-plugin

# In RViz: Add → rviz_imu_plugin → Imu
# Topic: /imu/data
```

### Add Marker Array

For waypoints or custom markers (requires publishing marker messages)

## Attribution

This package is based on the [nmea_navsat_driver](https://github.com/ros-drivers/nmea_navsat_driver) ROS2 package.

### License

BSD License - See the original package for full license terms.

The original `nmea_navsat_driver` package is Copyright (c) 2013, Eric Perko. All rights reserved.

## Links

- [Original ROS Wiki](http://ros.org/wiki/nmea_navsat_driver)
- [Original GitHub](https://github.com/ros-drivers/nmea_navsat_driver)
- [ROS 2 TF2 Documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)
- [RViz User Guide](https://github.com/ros2/rviz)
- [nav_msgs/Odometry Message](https://docs.ros2.org/latest/api/nav_msgs/msg/Odometry.html)
- [sensor_msgs/Imu Message](https://docs.ros2.org/latest/api/sensor_msgs/msg/Imu.html)

## Support

If you encounter issues:

1. Check all nodes are running: `ros2 node list`
2. Verify topics are publishing: `ros2 topic list`
3. Check TF tree: `ros2 run tf2_tools view_frames`
4. Enable debug logging: `--log-level debug`

For device-specific issues (GPS lock, INS calibration), refer to your Quectel device documentation.
