# Geodnet ROS2 NMEA NavSat Driver

A ROS2 driver for parsing NMEA sentences from GNSS/GPS devices and publishing standard ROS navigation messages. This package is specifically enhanced for Geodnet devices with integrated IMU support.

## Overview

This driver parses NMEA strings from GPS/GNSS devices and publishes ROS2 messages without requiring the GPSD daemon. It supports multiple connection methods (serial, UDP socket, TCP client) and includes extended support for proprietary Quectel NMEA sentences used by Geodnet devices.

## Features

- Standard NMEA sentence parsing (GGA, RMC, VTG)
- Proprietary Quectel sentence support:
  - `PQTMSENMSG` - Raw IMU data (accelerometer + gyroscope)
  - `PQTMDRPVA` - INS position, velocity, and attitude
- Connection interface:
  - Serial port
- Publishes standard ROS2 message types:
  - `sensor_msgs/NavSatFix` - GPS fix data
  - `sensor_msgs/Imu` - IMU data (raw and fused)
  - `geometry_msgs/TwistStamped` - Velocity
  - `nav_msgs/Odometry` - Full odometry from INS

## Installation

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

## Usage

### Serial Driver

For devices connected via serial port (USB, UART):

```bash
ros2 launch nmea_navsat_driver nmea_serial_driver.launch.py
```

Configuration (`config/nmea_serial_driver.yaml`):
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

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/fix` | `sensor_msgs/NavSatFix` | GPS fix with position and covariance |
| `/vel` | `geometry_msgs/TwistStamped` | Velocity from GPS/INS |
| `/time_reference` | `sensor_msgs/TimeReference` | GPS time reference |
| `/imu/data` | `sensor_msgs/Imu` | Fused IMU data with orientation |
| `/imu/data_raw` | `sensor_msgs/Imu` | Raw IMU data (accel + gyro only) |
| `/odometry/ins` | `nav_msgs/Odometry` | Full INS odometry |

## Supported NMEA Sentences

### Standard Sentences
- `GGA` - GPS Fix Data
- `RMC` - Recommended Minimum Navigation Information
- `VTG` - Track Made Good and Ground Speed

### Proprietary Sentences (Geodnet/Quectel)
- `PQTMSENMSG` - IMU sensor message (accelerometer, gyroscope, temperature)
- `PQTMDRPVA` - Dead reckoning position, velocity, and attitude

## Attribution

This package is based on the [nmea_navsat_driver](https://github.com/ros-drivers/nmea_navsat_driver) ROS2 package.

### License
BSD License - See the original package for full license terms.

The original `nmea_navsat_driver` package is Copyright (c) 2013, Eric Perko. All rights reserved.

## Links

- Original ROS Wiki: http://ros.org/wiki/nmea_navsat_driver
- Original GitHub: https://github.com/ros-drivers/nmea_navsat_driver
