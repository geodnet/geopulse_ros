# =============================================================================
# Geodnet ROS2 NMEA NavSat Driver
# =============================================================================
# Multi-stage build for a lean production image
#
# Build:  docker build -t geodnet-nmea-driver .
# Run:    docker run --rm --device=/dev/ttyUSB0 geodnet-nmea-driver
# =============================================================================

ARG ROS_DISTRO=humble

# -----------------------------------------------------------------------------
# Stage 1: Build
# -----------------------------------------------------------------------------
FROM ros:${ROS_DISTRO}-ros-base AS builder

WORKDIR /ros2_ws

# Install build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Copy package files for dependency resolution
COPY src/nmea_navsat_driver/package.xml src/nmea_navsat_driver/

# Install ROS dependencies via rosdep
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && apt-get update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*

# Copy full source
COPY src/ src/

# Build the workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --packages-up-to nmea_navsat_driver

# -----------------------------------------------------------------------------
# Stage 2: Runtime
# -----------------------------------------------------------------------------
FROM ros:${ROS_DISTRO}-ros-base AS runtime

ARG ROS_DISTRO=humble

WORKDIR /ros2_ws

# Install runtime dependencies only
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-serial \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*

# Install ROS runtime dependencies
COPY src/nmea_navsat_driver/package.xml /tmp/
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && apt-get update \
    && rosdep update \
    && rosdep install --from-paths /tmp --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/* /tmp/package.xml

# Copy built workspace from builder
COPY --from=builder /ros2_ws/install install/

# Copy config and launch files for easy access
COPY src/nmea_navsat_driver/config/ config/
COPY src/nmea_navsat_driver/launch/ launch/

# Setup entrypoint
COPY docker-entrypoint.sh /
RUN chmod +x /docker-entrypoint.sh

ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["ros2", "launch", "nmea_navsat_driver", "nmea_serial_driver.launch.py"]
