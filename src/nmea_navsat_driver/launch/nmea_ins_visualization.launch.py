#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare arguments
    port_arg = DeclareLaunchArgument("port", default_value="/dev/ttyUSB0", description="Serial port for NMEA device")

    baud_arg = DeclareLaunchArgument("baud", default_value="115200", description="Baud rate for serial communication")

    frame_id_arg = DeclareLaunchArgument(
        "frame_id", default_value="base_link", description="Frame ID for the GPS/IMU sensor"
    )

    # NMEA serial driver node
    nmea_driver_node = Node(
        package="nmea_navsat_driver",
        executable="nmea_serial_driver",
        name="nmea_serial_driver",
        output="screen",
        parameters=[
            {
                "port": LaunchConfiguration("port"),
                "baud": LaunchConfiguration("baud"),
                "frame_id": LaunchConfiguration("frame_id"),
                "useRMC": False,
            }
        ],
    )

    # TF broadcaster for odometry
    odom_tf_broadcaster = Node(
        package="nmea_navsat_driver", executable="odom_tf_broadcaster", name="odom_tf_broadcaster", output="screen"
    )

    # Static transform for visualization (map -> odom)
    static_tf_map_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_map_odom",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    )

    # RViz node for visualization
    rviz_node = Node(package="rviz2", executable="rviz2", name="rviz2", output="screen")

    return LaunchDescription(
        [
            port_arg,
            baud_arg,
            frame_id_arg,
            nmea_driver_node,
            odom_tf_broadcaster,
            static_tf_map_odom,
            rviz_node,
        ]
    )
