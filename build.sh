#!/bin/bash
# =============================================================================
# Build script for Geodnet ROS2 NMEA NavSat Driver
# =============================================================================
set -e

IMAGE_NAME="geodnet-nmea-driver"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

# =============================================================================
# Install udev rule for automatic device detection (one-time, requires sudo)
# Creates /dev/geodnet_nmea symlink for the GEODNET receiver (CH340, 1a86:55d2)
# =============================================================================
UDEV_RULE='SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d2", SYMLINK+="geodnet_nmea", MODE="0666"'
UDEV_FILE="/etc/udev/rules.d/99-geodnet.rules"

if [ ! -f "$UDEV_FILE" ] || ! grep -qF "$UDEV_RULE" "$UDEV_FILE"; then
    echo "Installing udev rule for GEODNET device detection..."
    echo "$UDEV_RULE" | sudo tee "$UDEV_FILE" > /dev/null
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo "udev rule installed. Device will appear as /dev/geodnet_nmea when connected."
else
    echo "udev rule already installed."
fi

# =============================================================================
# Build Docker image
# =============================================================================
echo "Building ${IMAGE_NAME}:${ROS_DISTRO}..."
docker build \
    --build-arg ROS_DISTRO="${ROS_DISTRO}" \
    -t "${IMAGE_NAME}:${ROS_DISTRO}" \
    -t "${IMAGE_NAME}:latest" \
    .

echo "Done. Run with: docker compose up"