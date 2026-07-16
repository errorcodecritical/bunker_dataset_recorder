#!/bin/bash
# Force the script to use the exact same ROS2 network environment
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/$ROS_DISTRO/setup.bash
source /ros2_ws/install/setup.bash

# 1. Check if the ROS2 daemon is alive and responding
if ! ros2 node list > /dev/null 2>&1; then
    exit 1
fi

# 2. Verify that the core Bunker base node is alive in the graph
# (Replace 'bunker_base_node' with the exact name seen in 'ros2 node list')
if ! ros2 node list | grep -q "bunker_base"; then
    exit 1
fi

# 3. Optional: Verify that the driver is actively publishing the telemetry state
# If no topics are published for 2 seconds, fail the check.
if ! timeout 2 ros2 topic echo --once /bunker_status > /dev/null 2>&1; then
    exit 1
fi

exit 0
