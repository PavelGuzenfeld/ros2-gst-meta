# ros2-gst-meta

ROS 2 ↔ GStreamer metadata bridge. Subscribes to **any** ROS 2 topic at runtime and attaches the serialized message as GStreamer buffer metadata.

[![CI](https://github.com/PavelGuzenfeld/ros2-gst-meta/actions/workflows/ci.yml/badge.svg)](https://github.com/PavelGuzenfeld/ros2-gst-meta/actions/workflows/ci.yml)

## Overview

This package provides two GStreamer elements that bridge ROS 2 topics into GStreamer pipelines using [gst-metadata](https://github.com/PavelGuzenfeld/gst-metadata):

| Element | Direction | Description |
|---------|-----------|-------------|
| `ros2attach` | ROS 2 → GStreamer | Subscribes to a topic, attaches serialized CDR as buffer metadata |
| `ros2detach` | GStreamer → ROS 2 | Reads metadata from buffers, publishes back to a ROS 2 topic |

Message types are **discovered at runtime** — no compile-time dependency on specific message packages.

## Sync Policies

The `ros2attach` element supports configurable sync strategies for pairing ROS 2 messages with video buffers:

| Policy | Behavior | Best for |
|--------|----------|----------|
| `latest` | Always attach the most recently received message | Real-time: IMU, GPS, temperature |
| `nearest` | Match by closest message timestamp to buffer PTS | Recording, offline processing |

Additional options:
- `max-age-ms` — discard messages older than N milliseconds (0 = unlimited)

## Example Pipeline

```bash
# Attach IMU data from ROS 2 to a video stream
gst-launch-1.0 \
    v4l2src ! videoconvert ! \
    ros2attach topic=/imu sync-policy=latest ! \
    ros2attach topic=/gps sync-policy=latest ! \
    ... ! fakesink

# Full round-trip: attach from ROS 2, pass through pipeline, publish back
gst-launch-1.0 \
    videotestsrc ! \
    ros2attach topic=/imu sync-policy=latest ! \
    identity ! \
    ros2detach topic=/imu/relayed msg-type=sensor_msgs/msg/Imu ! \
    fakesink
```

Multiple `ros2attach` elements can be chained — each adds its own metadata independently. Metadata from different topics coexists on the same buffer and is distinguished by topic hash.

## Building

Requires ROS 2 Jazzy and GStreamer 1.0 development packages.

```bash
# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Build with colcon
colcon build --packages-select ros2_gst_meta

# Or with cmake directly
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j$(nproc)
```

### Dependencies

- ROS 2 Jazzy (`rclcpp`)
- GStreamer 1.0:
  ```bash
  sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
  ```
- [gst-metadata](https://github.com/PavelGuzenfeld/gst-metadata) (fetched automatically via CMake FetchContent)

## Testing

```bash
# Unit tests (sync buffer + metadata round-trip, no ROS 2 runtime needed)
ctest --test-dir build --output-on-failure

# With sanitizers
cmake -B build-asan -DCMAKE_BUILD_TYPE=Debug -DROS2_GST_META_SANITIZER_ASAN=ON
cmake --build build-asan -j$(nproc)
ctest --test-dir build-asan --output-on-failure
```

## Element Properties

### ros2attach

| Property | Type | Default | Description |
|----------|------|---------|-------------|
| `topic` | string | `""` | ROS 2 topic to subscribe to |
| `sync-policy` | enum | `latest` | `latest` or `nearest` |
| `max-age-ms` | uint | `0` | Max message staleness in ms (0 = unlimited) |
| `node-name` | string | `gst_ros2_attach` | ROS 2 node name |

### ros2detach

| Property | Type | Default | Description |
|----------|------|---------|-------------|
| `topic` | string | `""` | ROS 2 topic to publish to |
| `msg-type` | string | `""` | Message type (e.g. `sensor_msgs/msg/Imu`) |
| `node-name` | string | `gst_ros2_detach` | ROS 2 node name |

## Architecture

```
ROS 2 topic ──subscription──▸ SyncBuffer ──pick()──▸ ros2attach ──▸ GstBuffer + Ros2MsgMeta
                                  │
                         push() from ROS callback
                         pick() from GStreamer streaming thread
                         mutex-protected, bounded deque
```

The serialized CDR payload (up to 4 KB) is stored as a fixed-size POD in the GStreamer buffer metadata system via `gst-metadata`'s CRTP `MetaBase`. This means:
- Zero heap allocation per buffer
- Metadata survives `videoconvert`, `videoscale`, `tee`, `queue`, etc.
- Multiple topics coexist on the same buffer
- Topic filtering via FNV-1a hash

## License

MIT
