from setuptools import setup, find_packages
import os
from glob import glob

package_name = "nmea_navsat_driver"

setup(
    name=package_name,
    version="2.0.1",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nithya Srinivasan",
    maintainer_email="nithya@geodnet.com",
    description="Package to parse NMEA strings and publish a very simple GPS message.",
    license="BSD",
    entry_points={
        "console_scripts": [
            "nmea_serial_driver = libnmea_navsat_driver.nodes.nmea_serial_driver:main",
            "odom_tf_broadcaster = libnmea_navsat_driver.nodes.odom_tf_broadcaster:main",
            "twist_tf_broadcaster = libnmea_navsat_driver.nodes.twist_tf_broadcaster:main",
        ],
    },
)
