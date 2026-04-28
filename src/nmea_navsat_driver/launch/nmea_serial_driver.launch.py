# Copyright 2018 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""A simple launch file for the nmea_serial_driver node."""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchIntrospector, LaunchService
from launch_ros import actions


def generate_launch_description():
    """Generate a launch description for a single serial driver."""
    config_file = os.path.join(
        get_package_share_directory("nmea_navsat_driver"),
        "config",
        "nmea_serial_driver.yaml",
    )

    # Allow environment variable overrides
    param_overrides = {}

    if os.environ.get("NMEA_BAUD"):
        param_overrides["baud"] = int(os.environ["NMEA_BAUD"])
    if os.environ.get("NMEA_PORT"):
        param_overrides["port"] = os.environ["NMEA_PORT"]
    if os.environ.get("NMEA_FRAME_ID"):
        param_overrides["frame_id"] = os.environ["NMEA_FRAME_ID"]

    # NTRIP overrides
    if os.environ.get("NTRIP_HOST"):
        param_overrides["ntrip_host"] = os.environ["NTRIP_HOST"]
    if os.environ.get("NTRIP_PORT"):
        param_overrides["ntrip_port"] = int(os.environ["NTRIP_PORT"])
    if os.environ.get("NTRIP_MOUNTPOINT"):
        param_overrides["ntrip_mountpoint"] = os.environ["NTRIP_MOUNTPOINT"]
    if os.environ.get("NTRIP_USERNAME"):
        param_overrides["ntrip_username"] = os.environ["NTRIP_USERNAME"]
    if os.environ.get("NTRIP_PASSWORD"):
        param_overrides["ntrip_password"] = os.environ["NTRIP_PASSWORD"]

    driver_node = actions.Node(
        package="nmea_navsat_driver",
        executable="nmea_serial_driver",
        output="screen",
        parameters=[config_file, param_overrides],
    )

    return LaunchDescription([driver_node])


def main(argv):
    ld = generate_launch_description()

    print("Starting introspection of launch description...")
    print("")

    print(LaunchIntrospector().format_launch_description(ld))

    print("")
    print("Starting launch of launch description...")
    print("")

    ls = LaunchService()
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == "__main__":
    main(sys.argv)