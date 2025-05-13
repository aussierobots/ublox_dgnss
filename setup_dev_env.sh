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
source install/setup.bash

echo "Development environment setup complete!"
echo "You can now run tests with: colcon test --packages-select ublox_dgnss_node --output-on-failure"
