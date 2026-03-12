# ros2-gst-meta

ROS 2 ↔ GStreamer metadata bridge. Subscribes to **any** ROS 2 topic at runtime and attaches the serialized message as GStreamer buffer metadata.

[![CI](https://github.com/PavelGuzenfeld/ros2-gst-meta/actions/workflows/ci.yml/badge.svg)](https://github.com/PavelGuzenfeld/ros2-gst-meta/actions/workflows/ci.yml)
[![Release](https://github.com/PavelGuzenfeld/ros2-gst-meta/releases/latest/badge.svg)](https://github.com/PavelGuzenfeld/ros2-gst-meta/releases/latest)

## Overview

This package provides three GStreamer elements that bridge ROS 2 topics into GStreamer pipelines using [gst-metadata](https://github.com/PavelGuzenfeld/gst-metadata):

| Element | Direction | Description |
|---------|-----------|-------------|
| `ros2attach` | ROS 2 → GStreamer | Subscribes to a topic, attaches serialized CDR as buffer metadata |
| `ros2detach` | GStreamer → ROS 2 | Reads metadata from buffers, publishes back to a ROS 2 topic |
| `ros2metaprint` | Debug | Prints attached ROS 2 metadata (topic hash, timestamps, hex payload) |

Message types are **discovered at runtime** — no compile-time dependency on specific message packages.

## Sync Policies

The `ros2attach` element supports configurable sync strategies for pairing ROS 2 messages with video buffers:

| Policy | Behavior | Best for |
|--------|----------|----------|
| `latest` | Always attach the most recently received message | Real-time: IMU, GPS, temperature |
| `nearest` | Match by closest message timestamp to buffer PTS | Recording, offline processing |

Additional options:
- `max-age-ms` — discard messages older than N milliseconds (0 = unlimited)

## Example Pipelines

```bash
# Attach IMU data from ROS 2 to a video stream and print metadata
gst-launch-1.0 \
    v4l2src ! videoconvert \
    ! ros2attach topic=/imu sync-policy=latest \
    ! ros2metaprint \
    ! fakesink

# Attach multiple topics
gst-launch-1.0 \
    videotestsrc \
    ! ros2attach topic=/imu node-name=gst_imu \
    ! ros2attach topic=/gps node-name=gst_gps \
    ! ros2metaprint verbose=true \
    ! fakesink

# Full round-trip: attach from ROS 2, print, publish back
gst-launch-1.0 \
    videotestsrc \
    ! ros2attach topic=/imu sync-policy=latest \
    ! ros2metaprint filter-topic=/imu \
    ! ros2detach topic=/imu/relayed msg-type=sensor_msgs/msg/Imu \
    ! fakesink
```

Multiple `ros2attach` elements can be chained — each adds its own metadata independently. Metadata from different topics coexists on the same buffer and is distinguished by FNV-1a topic hash.

See [`examples/`](examples/) for ready-to-run demo scripts.

## Building

Requires ROS 2 Jazzy and GStreamer 1.0 development packages.

```bash
# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Build with colcon
colcon build --packages-select ros2_gst_meta

# Or with cmake directly
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

### Dependencies

- ROS 2 Jazzy (`rclcpp`)
- GStreamer 1.0:
  ```bash
  sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
      gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good
  ```
- [gst-metadata](https://github.com/PavelGuzenfeld/gst-metadata) (fetched automatically via CMake FetchContent)

## Testing

### Unit Tests

Three test suites covering sync buffer logic, metadata round-trips, edge cases, and fuzz/stress (no ROS 2 runtime needed):

```bash
ctest --test-dir build --output-on-failure
```

### Sanitizer Builds

```bash
# ASan + UBSan
cmake -B build-asan -DCMAKE_BUILD_TYPE=Debug -DROS2_GST_META_SANITIZER_ASAN=ON
cmake --build build-asan -j$(nproc)
ctest --test-dir build-asan --output-on-failure

# TSan
cmake -B build-tsan -DCMAKE_BUILD_TYPE=Debug -DROS2_GST_META_SANITIZER_TSAN=ON
cmake --build build-tsan -j$(nproc)
ctest --test-dir build-tsan --output-on-failure
```

### E2E Integration Tests (Docker)

Full end-to-end tests with ROS 2 publishers and GStreamer pipelines:

```bash
docker build -t ros2-gst-meta-test .
docker run --rm ros2-gst-meta-test
```

Tests plugin loading, element registration, attach/print pipelines, round-trip attach→detach, multi-topic support, and graceful handling of missing publishers.

## Element Properties

### ros2attach

| Property | Type | Default | Description |
|----------|------|---------|-------------|
| `topic` | string | `""` | ROS 2 topic to subscribe to |
| `sync-policy` | enum | `latest` | `latest` or `nearest` |
| `max-age-ms` | uint | `0` | Max message staleness in ms (0 = unlimited) |
| `node-name` | string | `gst_ros2_attach` | ROS 2 node name |
| `qos-profile` | enum | `sensor` | `sensor`, `reliable`, or `system-default` |

### ros2detach

| Property | Type | Default | Description |
|----------|------|---------|-------------|
| `topic` | string | `""` | ROS 2 topic to publish to |
| `msg-type` | string | `""` | Message type (e.g. `sensor_msgs/msg/Imu`) |
| `node-name` | string | `gst_ros2_detach` | ROS 2 node name |
| `filter-topic` | string | `""` | Only publish metadata from this source topic (empty = all) |
| `qos-profile` | enum | `sensor` | `sensor`, `reliable`, or `system-default` |

### ros2metaprint

| Property | Type | Default | Description |
|----------|------|---------|-------------|
| `verbose` | bool | `false` | Print full serialized payload (not just first 16 bytes) |
| `filter-topic` | string | `""` | Only print metadata matching this topic (empty = all) |

## Architecture

```
ROS 2 topic ──GenericSubscription──▸ SyncBuffer ──pick()──▸ ros2attach ──▸ GstBuffer + Ros2MsgMeta
                                         │
                            push(): ROS 2 callback thread
                            pick(): GStreamer streaming thread
                            mutex-protected bounded deque
```

The serialized CDR payload (up to 4 KB) is stored as a fixed-size POD in the GStreamer buffer metadata system via `gst-metadata`'s CRTP `MetaBase`. This means:
- Zero heap allocation per buffer
- Metadata survives `videoconvert`, `videoscale`, `tee`, `queue`, etc.
- Multiple topics coexist on the same buffer
- Topic filtering via FNV-1a hash
- Metadata persists through buffer copies

## License

MIT — see [LICENSE](LICENSE) for details.
