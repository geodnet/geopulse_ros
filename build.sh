#!/bin/bash
# =============================================================================
# Build script for Geodnet ROS2 NMEA NavSat Driver
# =============================================================================

set -e

IMAGE_NAME="geodnet-nmea-driver"
ROS_DISTRO="${ROS_DISTRO:-humble}"

echo "Building ${IMAGE_NAME}:${ROS_DISTRO}..."

docker build \
    --build-arg ROS_DISTRO="${ROS_DISTRO}" \
    -t "${IMAGE_NAME}:${ROS_DISTRO}" \
    -t "${IMAGE_NAME}:latest" \
    .

echo "Done. Run with: docker compose up"
