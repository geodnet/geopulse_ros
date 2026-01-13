# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Eric Perko
# All rights reserved.

import math

from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference, Imu
from geometry_msgs.msg import TwistStamped, Quaternion, Vector3
from nav_msgs.msg import Odometry

from libnmea_navsat_driver.checksum_utils import check_nmea_checksum
import libnmea_navsat_driver.parser


def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles (roll, pitch, yaw) in radians to quaternion.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return (x, y, z, w)


class Ros2NMEADriver(object):
    """
    ROS2 driver for NMEA GNSS devices with IMU support.
    """

    G_TO_MS2 = 9.80665
    DEG_TO_RAD = math.pi / 180.0

    def __init__(self, frame_id="gps", time_ref_source=None, use_RMC=True):
        self.frame_id = frame_id
        self.time_ref_source = time_ref_source if time_ref_source != "" else None
        self.use_RMC = use_RMC

        self.fix_pub = None
        self.vel_pub = None
        self.time_ref_pub = None
        self.imu_data_pub = None
        self.imu_data_raw_pub = None
        self.odometry_pub = None

        self.current_fix = NavSatFix()
        self.current_fix.header.frame_id = self.frame_id
        self.current_fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.current_linear_accel = Vector3()
        self.current_angular_vel = Vector3()
        self.has_imu_raw_data = False

        self.current_orientation = Quaternion()
        self.current_orientation.w = 1.0
        self.has_orientation_data = False

        self.valid_fix = False

    def get_frame_id(self):
        return self.frame_id

    def add_sentence(self, nmea_string, frame_id, timestamp=None):
        """
        Parse an NMEA sentence and publish ROS2 messages.
        """
        if not nmea_string or not nmea_string.startswith("$"):
            return False

        try:
            nmea_string.encode("ascii")
        except UnicodeEncodeError:
            return False

        if not check_nmea_checksum(nmea_string):
            return False

        parsed_sentence = libnmea_navsat_driver.parser.parse_nmea_sentence(nmea_string)
        if not parsed_sentence:
            return False

        if frame_id:
            self.current_fix.header.frame_id = frame_id

        sentence_type = parsed_sentence["sentence_type"]

        if sentence_type == "GGA":
            self.handle_gga(parsed_sentence, timestamp)
            return True
        elif sentence_type == "RMC":
            self.handle_rmc(parsed_sentence, timestamp)
            return True
        elif sentence_type == "VTG":
            self.handle_vtg(parsed_sentence, timestamp)
            return True
        elif sentence_type == "PQTMSENMSG":
            self.handle_pqtmsenmsg(parsed_sentence, timestamp)
            return True
        elif sentence_type == "PQTMDRPVA":
            self.handle_pqtmdrpva(parsed_sentence, timestamp)
            return True

        return False

    def handle_gga(self, parsed_sentence, timestamp):
        """Handle GGA message - GPS Fix Data"""
        self.current_fix.header.stamp = timestamp

        if not math.isnan(parsed_sentence["latitude"]):
            self.current_fix.latitude = parsed_sentence["latitude"]
        if not math.isnan(parsed_sentence["longitude"]):
            self.current_fix.longitude = parsed_sentence["longitude"]
        if not math.isnan(parsed_sentence["altitude"]):
            self.current_fix.altitude = parsed_sentence["altitude"]

        fix_quality = parsed_sentence["fix_quality"]
        if fix_quality == 0:
            self.current_fix.status.status = NavSatStatus.STATUS_NO_FIX
        elif fix_quality == 1:
            self.current_fix.status.status = NavSatStatus.STATUS_FIX
        elif fix_quality == 2:
            self.current_fix.status.status = NavSatStatus.STATUS_SBAS_FIX
        elif fix_quality in [4, 5]:
            self.current_fix.status.status = NavSatStatus.STATUS_GBAS_FIX
        else:
            self.current_fix.status.status = NavSatStatus.STATUS_FIX

        self.current_fix.status.service = NavSatStatus.SERVICE_GPS

        hdop = parsed_sentence["hdop"]
        if not math.isnan(hdop):
            variance = math.pow(hdop * 5.0, 2)
            self.current_fix.position_covariance = [
                variance,
                0.0,
                0.0,
                0.0,
                variance,
                0.0,
                0.0,
                0.0,
                variance * 2,
            ]
            self.current_fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        if self.fix_pub:
            self.fix_pub.publish(self.current_fix)

        if self.time_ref_source and self.time_ref_pub:
            time_ref = TimeReference()
            time_ref.header.stamp = timestamp
            time_ref.header.frame_id = self.frame_id

            utc_time = parsed_sentence["utc_time"]
            if not math.isnan(utc_time):
                from builtin_interfaces.msg import Time as TimeMsg

                time_ref.time_ref = TimeMsg()
                time_ref.time_ref.sec = int(utc_time)
                time_ref.time_ref.nanosec = int((utc_time % 1) * 1e9)
                time_ref.source = self.time_ref_source
                self.time_ref_pub.publish(time_ref)

    def handle_rmc(self, parsed_sentence, timestamp):
        """Handle RMC message"""
        if not self.use_RMC:
            return

        self.current_fix.header.stamp = timestamp

        if parsed_sentence["fix_valid"]:
            if not math.isnan(parsed_sentence["latitude"]):
                self.current_fix.latitude = parsed_sentence["latitude"]
            if not math.isnan(parsed_sentence["longitude"]):
                self.current_fix.longitude = parsed_sentence["longitude"]

            self.current_fix.status.status = NavSatStatus.STATUS_FIX
            self.current_fix.status.service = NavSatStatus.SERVICE_GPS
        else:
            self.current_fix.status.status = NavSatStatus.STATUS_NO_FIX

        if self.fix_pub:
            self.fix_pub.publish(self.current_fix)

    def handle_vtg(self, parsed_sentence, timestamp):
        """Handle VTG message"""
        twist = TwistStamped()
        twist.header.stamp = timestamp
        twist.header.frame_id = self.frame_id

        if not math.isnan(parsed_sentence["speed"]):
            twist.twist.linear.x = parsed_sentence["speed"]

        if self.vel_pub:
            self.vel_pub.publish(twist)

    def handle_pqtmsenmsg(self, parsed_sentence, timestamp):
        """Handle PQTMSENMSG message - IMU Raw Data"""
        if parsed_sentence is None:
            return

        self.current_linear_accel.x = parsed_sentence["acc_x_g"] * self.G_TO_MS2
        self.current_linear_accel.y = parsed_sentence["acc_y_g"] * self.G_TO_MS2
        self.current_linear_accel.z = parsed_sentence["acc_z_g"] * self.G_TO_MS2

        self.current_angular_vel.x = parsed_sentence["gyro_x_deg"] * self.DEG_TO_RAD
        self.current_angular_vel.y = parsed_sentence["gyro_y_deg"] * self.DEG_TO_RAD
        self.current_angular_vel.z = parsed_sentence["gyro_z_deg"] * self.DEG_TO_RAD

        self.has_imu_raw_data = True

        if self.imu_data_raw_pub:
            msg = Imu()
            msg.header.stamp = timestamp
            msg.header.frame_id = self.frame_id

            msg.linear_acceleration = self.current_linear_accel
            msg.linear_acceleration_covariance = [
                0.01,
                0.0,
                0.0,
                0.0,
                0.01,
                0.0,
                0.0,
                0.0,
                0.01,
            ]

            msg.angular_velocity = self.current_angular_vel
            msg.angular_velocity_covariance = [
                0.000003,
                0.0,
                0.0,
                0.0,
                0.000003,
                0.0,
                0.0,
                0.0,
                0.000003,
            ]

            msg.orientation.x = 0.0
            msg.orientation.y = 0.0
            msg.orientation.z = 0.0
            msg.orientation.w = 1.0

            msg.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

            self.imu_data_raw_pub.publish(msg)

        if self.has_orientation_data and self.imu_data_pub:
            self.publish_full_imu(timestamp)

    def handle_pqtmdrpva(self, parsed_sentence, timestamp):
        """Handle PQTMDRPVA message - INS Data"""
        if parsed_sentence is None:
            return

        roll_rad = parsed_sentence["roll_deg"] * self.DEG_TO_RAD
        pitch_rad = parsed_sentence["pitch_deg"] * self.DEG_TO_RAD
        yaw_rad = parsed_sentence["heading_deg"] * self.DEG_TO_RAD

        qx, qy, qz, qw = euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)

        self.current_orientation.x = qx
        self.current_orientation.y = qy
        self.current_orientation.z = qz
        self.current_orientation.w = qw
        self.has_orientation_data = True

        if self.vel_pub:
            twist = TwistStamped()
            twist.header.stamp = timestamp
            twist.header.frame_id = self.frame_id
            twist.twist.linear.x = parsed_sentence["speed"]
            self.vel_pub.publish(twist)

        if self.has_imu_raw_data and self.imu_data_pub:
            self.publish_full_imu(timestamp)

        if self.odometry_pub:
            odom = Odometry()
            odom.header.stamp = timestamp
            odom.header.frame_id = "map"
            odom.child_frame_id = self.frame_id

            if not math.isnan(parsed_sentence["latitude"]) and not math.isnan(parsed_sentence["longitude"]):
                odom.pose.pose.position.x = parsed_sentence["latitude"]
                odom.pose.pose.position.y = parsed_sentence["longitude"]
                odom.pose.pose.position.z = (
                    parsed_sentence["altitude"] if not math.isnan(parsed_sentence["altitude"]) else 0.0
                )

            odom.pose.pose.orientation = self.current_orientation

            sol_type = parsed_sentence["solution_type"]
            if sol_type == 4:
                pos_variance = 0.04
            elif sol_type == 5:
                pos_variance = 1.0
            elif sol_type == 2:
                pos_variance = 4.0
            elif sol_type == 1:
                pos_variance = 25.0
            else:
                pos_variance = 10000.0

            odom.pose.covariance = [
                pos_variance,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                pos_variance,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                pos_variance * 4,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0087**2,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0087**2,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.017**2,
            ]

            odom.twist.twist.linear.x = parsed_sentence["vel_north"]
            odom.twist.twist.linear.y = parsed_sentence["vel_east"]
            odom.twist.twist.linear.z = -parsed_sentence["vel_down"]

            if self.has_imu_raw_data:
                odom.twist.twist.angular = self.current_angular_vel
            else:
                odom.twist.twist.angular.x = 0.0
                odom.twist.twist.angular.y = 0.0
                odom.twist.twist.angular.z = 0.0

            odom.twist.covariance = [
                0.1,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.1,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.1,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.000003,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.000003,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.000003,
            ]

            self.odometry_pub.publish(odom)

    def publish_full_imu(self, timestamp):
        """Publish full 9-DOF IMU message"""
        if not self.imu_data_pub:
            return

        msg = Imu()
        msg.header.stamp = timestamp
        msg.header.frame_id = self.frame_id

        msg.orientation = self.current_orientation
        msg.orientation_covariance = [
            0.0087**2,
            0.0,
            0.0,
            0.0,
            0.0087**2,
            0.0,
            0.0,
            0.0,
            0.017**2,
        ]

        msg.linear_acceleration = self.current_linear_accel
        msg.linear_acceleration_covariance = [
            0.01,
            0.0,
            0.0,
            0.0,
            0.01,
            0.0,
            0.0,
            0.0,
            0.01,
        ]

        msg.angular_velocity = self.current_angular_vel
        msg.angular_velocity_covariance = [
            0.000003,
            0.0,
            0.0,
            0.0,
            0.000003,
            0.0,
            0.0,
            0.0,
            0.000003,
        ]

        self.imu_data_pub.publish(msg)
