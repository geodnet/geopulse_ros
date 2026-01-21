#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, TransformStamped
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
import math


class TwistTFBroadcaster(Node):
    """
    Broadcasts TF transform by integrating twist (velocity) data in real-time.
    Shows the reference frame moving based on linear and angular velocities.
    """

    def __init__(self):
        super().__init__("twist_tf_broadcaster")

        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Current pose (integrated from velocities)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        # Current orientation (quaternion)
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self.qw = 1.0

        # Current angular velocity (for integration)
        self.angular_vel_x = 0.0
        self.angular_vel_y = 0.0
        self.angular_vel_z = 0.0

        # Time tracking
        self.last_time = None

        # Subscribe to velocity topic
        self.vel_sub = self.create_subscription(TwistStamped, "vel", self.vel_callback, 10)

        # Subscribe to IMU for orientation (optional - for better orientation tracking)
        self.imu_sub = self.create_subscription(Imu, "imu/data", self.imu_callback, 10)

        # Timer to broadcast TF at fixed rate
        self.timer = self.create_timer(0.02, self.broadcast_tf)  # 50 Hz

        self.get_logger().info("Twist TF Broadcaster started - frame will move in real-time!")

    def vel_callback(self, msg):
        """Callback to integrate velocity into position."""
        current_time = self.get_clock().now()

        if self.last_time is not None:
            # Calculate dt
            dt = (current_time - self.last_time).nanoseconds / 1e9

            if dt > 0 and dt < 1.0:  # Sanity check
                # Get velocities
                vx = msg.twist.linear.x
                vy = msg.twist.linear.y
                vz = msg.twist.linear.z

                # Store angular velocities
                self.angular_vel_x = msg.twist.angular.x
                self.angular_vel_y = msg.twist.angular.y
                self.angular_vel_z = msg.twist.angular.z

                # Integrate position
                # Note: These velocities are in the body frame, but for visualization
                # we're integrating them directly. For accurate odometry, you'd need
                # to rotate by current orientation first.
                self.x += vx * dt
                self.y += vy * dt
                self.z += vz * dt

                # Integrate orientation (simple integration)
                # Convert current quaternion to euler for integration
                roll, pitch, yaw = self.quaternion_to_euler(self.qx, self.qy, self.qz, self.qw)

                # Integrate angular velocities
                roll += self.angular_vel_x * dt
                pitch += self.angular_vel_y * dt
                yaw += self.angular_vel_z * dt

                # Convert back to quaternion
                self.qx, self.qy, self.qz, self.qw = self.euler_to_quaternion(roll, pitch, yaw)

        self.last_time = current_time

    def imu_callback(self, msg):
        """Use IMU orientation directly instead of integrating."""
        # Override integrated orientation with IMU orientation
        # This is more accurate than integration
        self.qx = msg.orientation.x
        self.qy = msg.orientation.y
        self.qz = msg.orientation.z
        self.qw = msg.orientation.w

    def broadcast_tf(self):
        """Broadcast the current TF transform."""
        t = TransformStamped()

        # Set header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        # Set transform
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = self.z

        t.transform.rotation.x = self.qx
        t.transform.rotation.y = self.qy
        t.transform.rotation.z = self.qz
        t.transform.rotation.w = self.qw

        # Broadcast transform
        self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """Convert Euler angles to quaternion."""
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

    @staticmethod
    def quaternion_to_euler(x, y, z, w):
        """Convert quaternion to Euler angles."""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return (roll, pitch, yaw)


def main(args=None):
    rclpy.init(args=args)

    node = TwistTFBroadcaster()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
