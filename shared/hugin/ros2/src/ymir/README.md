# Copyright (c) Sensrad 2025

# Ymir - Multi-Radar Odometry Aggregator

Ymir is a ROS2 node that aggregates data from multiple radars using event-driven state-based fusion. In the current version only odometry inoformation is fused. But the code serves
as example code for how to aggregate data from multiple radar.

## Overview

Ymir subscribes to `ego_motion` messages from one or more radar namespaces and publishes:
- An aggregated `odom → base_link` TF transform
- An `ego_trajectory` path message on `/sensrad/ymir/ego_trajectory`

The node uses an event-driven approach where each incoming radar measurement immediately updates the fused state and publishes the result.

Ymir automatically subscribed to all topics matching the name-space structure /sensrad/radar_x/

## Architecture

```
Multiple Radars:
  /sensrad/radar_1/oden/ego_motion  ─┐
  /sensrad/radar_2/oden/ego_motion  ─┤  Event-driven
  /sensrad/radar_N/oden/ego_motion  ─┘  (no buffering)
           │
           ▼
    ┌─────────────┐
    │    Ymir     │  Single radar: Direct output
    │  (callback) │  Multi-radar: Low-pass filter
    └─────────────┘
           │
           ├─► TF: odom → base_link
           └─► Topic: /sensrad/ymir/ego_trajectory
```

## Fusion Strategy

The fusion process adapts based on the number of active radars:

### Single Radar Mode
- **Direct output**: Measurements are published immediately without filtering
- **Zero latency**: No averaging or smoothing applied
- **Use case**: When only one radar is available or configured

### Multi-Radar Mode
- **Low-pass filtering**: Incoming measurements are blended with the current state
- **State update**: Uses exponential filtering with configurable alpha coefficient
  - **Translation**: `filtered = α × new + (1-α) × current`
  - **Rotation**: SLERP (Spherical Linear Interpolation) between current and new quaternion
- **Smoothness**: Higher alpha = more responsive, lower alpha = smoother output

### Event-Driven Operation
1. **Immediate processing**: Each incoming `ego_motion` message triggers the callback
2. **Transform lookup**: Radar-to-baselink transform is retrieved (cached for efficiency)
3. **Pose computation**: Measurement is transformed to base_link coordinate system
4. **State update**: Single radar → direct copy, multi-radar → low-pass filter
5. **Immediate publish**: TF and trajectory are published without delay

## Configuration

See [config/ymir.yaml](config/ymir.yaml) for configuration options

### Parameters

- **`alpha`** (default: 0.3): Low-pass filter coefficient (0 < alpha ≤ 1.0)
  - **Only used when multiple radars are present** (single radar uses direct output)
  - Higher values = more responsive to new measurements
  - Lower values = smoother, more filtered output
  - Typical values:
    - `0.1`: Very smooth, slow response (high filtering)
    - `0.3`: Balanced (default)
    - `0.5`: More responsive, less filtering

- **`odom_frame_id`** (default: `"odom"`): Parent frame for the odometry TF
- **`baselink_frame_id`** (default: `"base_link"`): Child frame for the odometry TF

## Launch

### Standalone
```bash
# Single radar
ros2 launch ymir ymir.launch.py"
```

### Integration
Ymir is automatically launched by `stack.launch.py` for all radar setups. The radar namespaces are built dynamically from the `radar_count` parameter.

## Design Principles

- **Event-driven**: No periodic timers, publishes immediately when data arrives
- **No buffering**: Minimal latency, no message queuing
- **Adaptive fusion**: Behavior changes automatically based on number of radars
- **Simple state management**: Single filtered state for multi-radar case
- **Configurable responsiveness**: Alpha parameter allows tuning the filter behavior
- **Single source of truth**: Radar configuration specified once in launch files

### Published
- `/sensrad/ymir/ego_trajectory` (nav_msgs/msg/Path): Aggregated trajectory path
- `/tf` (tf2_msgs/msg/TFMessage): TF transform from `odom` to `base_link`

## Frame Requirements

Ymir requires the following TF transforms to be available:
- `radar_N → base_link`: Static transforms for each radar (published by static_tf_publisher)
