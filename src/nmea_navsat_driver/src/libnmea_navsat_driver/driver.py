# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Eric Perko
# All rights reserved.

import math

from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference, Imu
from geometry_msgs.msg import TwistStamped, Quaternion, Vector3
from nav_msgs.msg import Odometry
from nmea_driver_msgs.msg import Rtcm

from libnmea_navsat_driver.checksum_utils import check_nmea_checksum
import libnmea_navsat_driver.parser

from builtin_interfaces.msg import Time as TimeMsg
from std_msgs.msg import Int8


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


def geodetic_to_enu(lat, lon, alt, lat_origin, lon_origin, alt_origin):
    """
    Convert geodetic coordinates (lat, lon, alt) to ENU (East, North, Up)
    coordinates relative to an origin point.
    """
    # WGS84 parameters
    a = 6378137.0  # Semi-major axis
    f = 1.0 / 298.257223563  # Flattening
    e2 = 2 * f - f * f  # Square of eccentricity
    
    # Convert to radians
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    lat_origin_rad = math.radians(lat_origin)
    lon_origin_rad = math.radians(lon_origin)
    
    # Calculate radius of curvature in prime vertical
    N_origin = a / math.sqrt(1 - e2 * math.sin(lat_origin_rad)**2)
    N = a / math.sqrt(1 - e2 * math.sin(lat_rad)**2)
    
    # Convert to ECEF (Earth-Centered, Earth-Fixed)
    x_origin = (N_origin + alt_origin) * math.cos(lat_origin_rad) * math.cos(lon_origin_rad)
    y_origin = (N_origin + alt_origin) * math.cos(lat_origin_rad) * math.sin(lon_origin_rad)
    z_origin = (N_origin * (1 - e2) + alt_origin) * math.sin(lat_origin_rad)
    
    x = (N + alt) * math.cos(lat_rad) * math.cos(lon_rad)
    y = (N + alt) * math.cos(lat_rad) * math.sin(lon_rad)
    z = (N * (1 - e2) + alt) * math.sin(lat_rad)
    
    # Calculate difference in ECEF
    dx = x - x_origin
    dy = y - y_origin
    dz = z - z_origin
    
    # Rotation matrix from ECEF to ENU
    sin_lat = math.sin(lat_origin_rad)
    cos_lat = math.cos(lat_origin_rad)
    sin_lon = math.sin(lon_origin_rad)
    cos_lon = math.cos(lon_origin_rad)
    
    east = -sin_lon * dx + cos_lon * dy
    north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
    
    return (east, north, up)


class Ros2NMEADriver(object):
    """
    ROS2 driver for NMEA GNSS devices with IMU support.
    """

    G_TO_MS2 = 9.80665
    DEG_TO_RAD = math.pi / 180.0

    def __init__(self, frame_id="gps", time_ref_source=None):
        self.frame_id = frame_id
        self.time_ref_source = time_ref_source if time_ref_source != "" else None

        self.fix_pub = None
        self.vel_pub = None
        self.time_ref_pub = None
        self.imu_data_pub = None
        self.imu_data_raw_pub = None
        self.odometry_pub = None
        self.rtcm_pub = None
        self.nmea_status_pub = None

        self.current_fix = NavSatFix()
        self.current_fix.header.frame_id = self.frame_id
        self.current_fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        # Track the latest UTC time for messages that don't have their own timestamp
        self.latest_utc_time = TimeMsg(sec=0, nanosec=0)

        # IMU data - NO DEFAULT VALUES, only set from device
        self.current_linear_accel = None
        self.current_angular_vel = None
        self.has_imu_raw_data = False

        # Orientation data - NO DEFAULT VALUES, only set from device
        self.current_orientation = None
        self.has_orientation_data = False

        self.valid_fix = False
        
        # ENU origin (will be set from first valid RTK position)
        self.enu_origin_lat = None
        self.enu_origin_lon = None
        self.enu_origin_alt = None

    def get_frame_id(self):
        return self.frame_id

    def add_sentence(self, nmea_string, frame_id):
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
            self.handle_gga(parsed_sentence)
            return True
        elif sentence_type == "VTG":
            self.handle_vtg(parsed_sentence)
            return True
        elif sentence_type == "PQTMSENMSG":
            self.handle_pqtmsenmsg(parsed_sentence)
            return True
        elif sentence_type == "PQTMDRPVA":
            self.handle_pqtmdrpva(parsed_sentence)
            return True

        return False

    def handle_gga(self, parsed_sentence):
        """Handle GGA message - GPS Fix Data"""
        utc_time = parsed_sentence["utc_time"]
        if not math.isnan(utc_time):
            self.latest_utc_time = self.convert_gps_time_to_ros(utc_time)
            self.current_fix.header.stamp = self.latest_utc_time
        else:
            # Cannot timestamp strictly with UTC if no UTC time is present
            return 

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

        if self.nmea_status_pub:
            raw_status_msg = Int8()
            raw_status_msg.data = fix_quality
            self.nmea_status_pub.publish(raw_status_msg)

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
            time_ref.header.stamp = self.latest_utc_time
            time_ref.header.frame_id = self.frame_id

            time_ref.time_ref = TimeMsg()
            time_ref.time_ref.sec = int(utc_time)
            time_ref.time_ref.nanosec = int((utc_time % 1) * 1e9)
            time_ref.source = self.time_ref_source
            self.time_ref_pub.publish(time_ref)

    def handle_vtg(self, parsed_sentence):
        """Handle VTG message"""
        # VTG doesn't contain time, rely strictly on latest UTC
        if self.latest_utc_time.sec == 0 and self.latest_utc_time.nanosec == 0:
            return # Drop if we haven't received a valid UTC time yet

        twist = TwistStamped()
        twist.header.stamp = self.latest_utc_time
        twist.header.frame_id = self.frame_id

        if not math.isnan(parsed_sentence["speed"]):
            twist.twist.linear.x = parsed_sentence["speed"]

        if self.vel_pub:
            self.vel_pub.publish(twist)

    def handle_pqtmsenmsg(self, parsed_sentence):
        """Handle PQTMSENMSG message - IMU Raw Data"""
        if parsed_sentence is None:
            return

        # Rely strictly on latest UTC for IMU time (raw IMU has internal ms, converted to absolute UTC)
        if self.latest_utc_time.sec == 0 and self.latest_utc_time.nanosec == 0:
            return

        accel = Vector3()
        accel.x = parsed_sentence["acc_x_g"] * self.G_TO_MS2      # forward (same)
        accel.y = -parsed_sentence["acc_y_g"] * self.G_TO_MS2     # right -> left (negate)
        accel.z = -parsed_sentence["acc_z_g"] * self.G_TO_MS2     # down -> up (negate)
        
        gyro = Vector3()
        gyro.x = parsed_sentence["gyro_x_deg"] * self.DEG_TO_RAD   # roll rate (same)
        gyro.y = -parsed_sentence["gyro_y_deg"] * self.DEG_TO_RAD  # pitch rate (negate)
        gyro.z = -parsed_sentence["gyro_z_deg"] * self.DEG_TO_RAD  # yaw rate (negate)

        # Only update if we have valid data
        self.current_linear_accel = accel
        self.current_angular_vel = gyro
        self.has_imu_raw_data = True

        # Publish raw IMU (no orientation)
        if self.imu_data_raw_pub:
            msg = Imu()
            msg.header.stamp = self.latest_utc_time
            msg.header.frame_id = self.frame_id

            msg.linear_acceleration = self.current_linear_accel
            msg.linear_acceleration_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01,
            ]

            msg.angular_velocity = self.current_angular_vel
            msg.angular_velocity_covariance = [
                0.000003, 0.0, 0.0,
                0.0, 0.000003, 0.0,
                0.0, 0.0, 0.000003,
            ]

            msg.orientation.x = 0.0
            msg.orientation.y = 0.0
            msg.orientation.z = 0.0
            msg.orientation.w = 0.0
            msg.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

            self.imu_data_raw_pub.publish(msg)

        # Publish full IMU if we also have orientation
        if self.has_orientation_data and self.imu_data_pub:
            self.publish_full_imu()

    def handle_pqtmdrpva(self, parsed_sentence):
        """Handle PQTMDRPVA message - INS Data"""
        if parsed_sentence is None:
            return
            
        utc_time = parsed_sentence["utc_time"]
        if not math.isnan(utc_time):
            self.latest_utc_time = self.convert_gps_time_to_ros(utc_time)
        elif self.latest_utc_time.sec == 0 and self.latest_utc_time.nanosec == 0:
            return # Cannot proceed without UTC time
            
        roll_ned = parsed_sentence["roll_deg"] * self.DEG_TO_RAD
        pitch_ned = parsed_sentence["pitch_deg"] * self.DEG_TO_RAD
        heading_rad = parsed_sentence["heading_deg"] * self.DEG_TO_RAD
        
        roll_flu = -roll_ned
        pitch_flu = pitch_ned
        
        yaw_enu = (math.pi / 2) - heading_rad
        yaw_enu = (yaw_enu + math.pi) % (2 * math.pi) - math.pi
        
        qx, qy, qz, qw = euler_to_quaternion(roll_flu, pitch_flu, yaw_enu)
        
        orientation = Quaternion()
        orientation.x = qx
        orientation.y = qy
        orientation.z = qz
        orientation.w = qw
        
        self.current_orientation = orientation
        self.has_orientation_data = True
        
        # Publish velocity from PQTMDRPVA
        if self.vel_pub:
            twist = TwistStamped()
            twist.header.stamp = self.latest_utc_time
            twist.header.frame_id = self.frame_id
            twist.twist.linear.x = parsed_sentence["vel_east"]   
            twist.twist.linear.y = parsed_sentence["vel_north"]  
            twist.twist.linear.z = -parsed_sentence["vel_down"]  
            if self.has_imu_raw_data:
                twist.twist.angular = self.current_angular_vel
            self.vel_pub.publish(twist)

        # Publish full IMU data if available
        if self.has_imu_raw_data and self.imu_data_pub:
            self.publish_full_imu()

        # Publish odometry with ENU coordinates
        if not self.odometry_pub:
            return
        
        odom = Odometry()
        odom.header.stamp = self.latest_utc_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = self.frame_id
        
        # Set orientation (in ENU frame)
        odom.pose.pose.orientation = self.current_orientation
        
        lat = parsed_sentence["latitude"]
        lon = parsed_sentence["longitude"]
        alt = parsed_sentence["altitude"]
        sol_type = parsed_sentence["solution_type"]
        
        if not (math.isnan(lat) or math.isnan(lon) or math.isnan(alt)):
            if self.enu_origin_lat is None and sol_type in [4, 5]:
                self.enu_origin_lat = lat
                self.enu_origin_lon = lon
                self.enu_origin_alt = alt
            
            if self.enu_origin_lat is not None:
                east, north, up = geodetic_to_enu(
                    lat, lon, alt,
                    self.enu_origin_lat, self.enu_origin_lon, self.enu_origin_alt
                )
                odom.pose.pose.position.x = east
                odom.pose.pose.position.y = north
                odom.pose.pose.position.z = up

        if sol_type == 4:  # RTK Fixed
            pos_variance = 0.04
        elif sol_type == 5:  # RTK Float
            pos_variance = 1.0
        elif sol_type == 2:  # DGPS
            pos_variance = 4.0
        elif sol_type == 1:  # Single point
            pos_variance = 25.0
        else:
            pos_variance = 10000.0

        odom.pose.covariance = [
            pos_variance, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, pos_variance, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, pos_variance * 4, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0087**2, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0087**2, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.017**2,
        ]

        odom.twist.twist.linear.x = parsed_sentence["vel_east"]   
        odom.twist.twist.linear.y = parsed_sentence["vel_north"]  
        odom.twist.twist.linear.z = -parsed_sentence["vel_down"]  

        if self.has_imu_raw_data:
            odom.twist.twist.angular = self.current_angular_vel
        else:
            odom.twist.twist.angular.x = 0.0
            odom.twist.twist.angular.y = 0.0
            odom.twist.twist.angular.z = 0.0

        odom.twist.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.000003, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.000003, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.000003,
        ]

        self.odometry_pub.publish(odom)

    def publish_full_imu(self):
        """Publish full 9-DOF IMU message with orientation"""
        if not self.imu_data_pub or not self.has_orientation_data or not self.has_imu_raw_data:
            return

        msg = Imu()
        msg.header.stamp = self.latest_utc_time
        msg.header.frame_id = self.frame_id

        msg.orientation = self.current_orientation
        msg.orientation_covariance = [
            0.0087**2, 0.0, 0.0,
            0.0, 0.0087**2, 0.0,
            0.0, 0.0, 0.017**2,
        ]

        msg.linear_acceleration = self.current_linear_accel
        msg.linear_acceleration_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01,
        ]

        msg.angular_velocity = self.current_angular_vel
        msg.angular_velocity_covariance = [
            0.000003, 0.0, 0.0,
            0.0, 0.000003, 0.0,
            0.0, 0.0, 0.000003,
        ]

        self.imu_data_pub.publish(msg)

    def handle_rtcm(self, rtcm_bytes):
        """Handle RTCM binary message - validate and publish to topic with GPS epoch time"""
        if len(rtcm_bytes) < 6 or rtcm_bytes[0] != 0xD3:
            return False
        
        msg_type = (rtcm_bytes[3] << 4) | (rtcm_bytes[4] >> 4)
        
        device_timestamp = None
        if len(rtcm_bytes) >= 11 and msg_type >= 1071:
            epoch_time_ms = (
                ((rtcm_bytes[6] & 0x0F) << 26) |
                (rtcm_bytes[7] << 18) |
                (rtcm_bytes[8] << 10) |
                (rtcm_bytes[9] << 2) |
                ((rtcm_bytes[10] & 0xC0) >> 6)
            )
            gps_seconds = epoch_time_ms / 1000.0
            device_timestamp = TimeMsg()
            device_timestamp.sec = int(gps_seconds)
            device_timestamp.nanosec = int((gps_seconds % 1) * 1e9)
        
        if self.rtcm_pub:
            # UPDATED: Use Rtcm custom message
            msg = Rtcm()
            msg.header.frame_id = self.frame_id
            msg.header.stamp = device_timestamp if device_timestamp else TimeMsg()
            msg.data = list(rtcm_bytes)  # Convert bytes to list for uint8[] field
            self.rtcm_pub.publish(msg)
        
        return True
    
    def convert_gps_time_to_ros(self, utc_time_seconds):
        """Convert GPS UTC time (seconds since midnight) to ROS timestamp"""
        timestamp = TimeMsg()
        timestamp.sec = int(utc_time_seconds)
        timestamp.nanosec = int((utc_time_seconds % 1) * 1e9)
        return timestamp