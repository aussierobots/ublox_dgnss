#!/bin/bash
# Development environment setup script for ublox_dgnss

# Source ROS environment
source /opt/ros/jazzy/setup.bash

# Clean build if requested
if [ "$1" == "clean" ]; then
  echo "Performing clean build..."
  rm -rf build/ install/ log/
fi

# Build the workspace
echo "Building workspace..."
colcon build --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON

# Source the workspace setup
source ~/ros2_ws/install/setup.bash

# Check if the package-specific setup file exists before trying to source it
UBLOX_SETUP="/home/geoff/ros2_ws/install/ublox_dgnss_node/share/ublox_dgnss_node/local_setup.bash"
if [ -f "$UBLOX_SETUP" ]; then
  source "$UBLOX_SETUP"
else
  echo "Note: Package setup file not found at $UBLOX_SETUP"
  echo "This is normal if the package has not been built successfully yet."
fi

echo "Development environment setup complete!"
echo "You can now run tests with: colcon test --packages-select ublox_dgnss_node --output-on-failure"
