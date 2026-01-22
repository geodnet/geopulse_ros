#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Eric Perko
# All rights reserved.

import serial
import sys

import rclpy
from rclpy.node import Node

from libnmea_navsat_driver.driver import Ros2NMEADriver

from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference, Imu
from geometry_msgs.msg import TwistStamped

from nav_msgs.msg import Odometry

from std_msgs.msg import UInt8MultiArray


class NMEASerialNode(Node):
    """ROS2 node for reading NMEA data from a serial port."""

    def __init__(self):
        super().__init__("nmea_serial_driver")

        # Instance variable for reading raw bytes
        self.buffer = bytearray()

        # Declare parameters
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("frame_id", "gps")
        self.declare_parameter("time_ref_source", "")
        self.declare_parameter("useRMC", False)

        # Get parameters
        port = self.get_parameter("port").value
        baud = self.get_parameter("baud").value
        frame_id = self.get_parameter("frame_id").value
        time_ref_source = self.get_parameter("time_ref_source").value
        use_rmc = self.get_parameter("useRMC").value

        # Create driver
        self.driver = Ros2NMEADriver(frame_id=frame_id, time_ref_source=time_ref_source, use_RMC=use_rmc)

        # Create publishers and assign to driver
        self.driver.fix_pub = self.create_publisher(NavSatFix, "fix", 10)
        self.driver.vel_pub = self.create_publisher(TwistStamped, "vel", 10)
        self.driver.time_ref_pub = self.create_publisher(TimeReference, "time_reference", 10)
        self.driver.imu_data_pub = self.create_publisher(Imu, "imu/data", 10)
        self.driver.imu_data_raw_pub = self.create_publisher(Imu, "imu/data_raw", 10)
        self.driver.odometry_pub = self.create_publisher(Odometry, "odometry/ins", 10)
        self.driver.rtcm_pub = self.create_publisher(UInt8MultiArray, "rtcm", 10)

        # Open serial port
        try:
            self.serial_port = serial.Serial(port=port, baudrate=baud, timeout=0.1)
            self.get_logger().info(f"Opened serial port {port} at {baud} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Could not open serial port {port}: {e}")
            raise

        # Create timer to read from serial port
        # Check for data every 10ms (100 Hz)
        self.timer = self.create_timer(0.01, self.read_serial)

        self.get_logger().info(f"NMEA Driver initialized with frame_id: {frame_id}")

    def read_serial(self):
        """Read data from serial port and process NMEA sentences."""
        try:
            # Read available data into buffer
            if self.serial_port.in_waiting > 0:
                self.buffer.extend(self.serial_port.read(self.serial_port.in_waiting))

            # Process buffer - extract NMEA and RTCM messages
            while len(self.buffer) > 0:
                if self.buffer[0] == ord('$'):  # NMEA sentence
                    # Find newline
                    newline_idx = self.buffer.find(b'\n')
                    if newline_idx == -1:
                        break  # Incomplete sentence
                    sentence = self.buffer[:newline_idx].decode('ascii', errors='ignore').strip()
                    self.buffer = self.buffer[newline_idx+1:]
                    if not sentence:
                        continue
                    
                    # Get current timestamp
                    timestamp = self.get_clock().now().to_msg()
                    
                    # Process NMEA sentence
                    try:
                        processed = self.driver.add_sentence(sentence, self.driver.get_frame_id(), timestamp)
                        if processed:
                            self.get_logger().debug(f"Processed: {sentence[:50]}")
                    except ValueError as e:
                        self.get_logger().warning(f"Error parsing sentence: {e}")
                    except Exception as e:
                        self.get_logger().error(f"Unexpected error: {e}")
                    
                elif self.buffer[0] == 0xD3:  # RTCM message
                    # Check if we have enough bytes for header (3 bytes minimum)
                    if len(self.buffer) < 3:
                        break
                    # Extract length from RTCM header (bits 14-23 of first 3 bytes)
                    length = ((self.buffer[1] & 0x03) << 8) | self.buffer[2]
                    msg_length = 3 + length + 3  # preamble + payload + CRC
                    if len(self.buffer) < msg_length:
                        break  # Incomplete message
                    rtcm_msg = bytes(self.buffer[:msg_length])
                    self.buffer = self.buffer[msg_length:]
                    timestamp = self.get_clock().now().to_msg()
                    # Printing to debug
                    print(f"Extracted RTCM from buffer: {len(rtcm_msg)} bytes, first 6: {' '.join([f'{b:02X}' for b in rtcm_msg[:6]])}")
                    self.driver.add_rtcm_message(rtcm_msg, timestamp)
                    
                else:
                    # Unknown byte, skip it
                    self.buffer = self.buffer[1:]

        except serial.SerialException as e:
            self.get_logger().error(f"Serial port error: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error reading serial: {e}")

    def destroy_node(self):
        """Clean up resources."""
        if hasattr(self, "serial_port") and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info("Closed serial port")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = NMEASerialNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1
    finally:
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
