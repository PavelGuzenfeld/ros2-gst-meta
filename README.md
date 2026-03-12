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

## QoS Profiles

Both `ros2attach` and `ros2detach` support configurable ROS 2 QoS profiles:

| Profile | Settings | Use case |
|---------|----------|----------|
| `sensor` (default) | Best-effort, volatile, keep-last(5) | High-frequency sensor data (IMU, GPS) |
| `reliable` | Reliable, volatile, keep-last(10) | Commands, state updates |
| `system-default` | Uses ROS 2 system defaults | Interop with existing nodes |

## Example Pipelines

```bash
# Attach IMU data from ROS 2 to a video stream and print metadata
gst-launch-1.0 \
    v4l2src ! videoconvert \
    ! ros2attach topic=/imu sync-policy=latest \
    ! ros2metaprint \
    ! fakesink

# Attach multiple topics with different QoS
gst-launch-1.0 \
    videotestsrc \
    ! ros2attach topic=/imu node-name=gst_imu qos-profile=sensor \
    ! ros2attach topic=/cmd node-name=gst_cmd qos-profile=reliable \
    ! ros2metaprint verbose=true \
    ! fakesink

# Full round-trip: attach from ROS 2, print, publish back
gst-launch-1.0 \
    videotestsrc \
    ! ros2attach topic=/imu sync-policy=latest \
    ! ros2metaprint filter-topic=/imu \
    ! ros2detach topic=/imu/relayed msg-type=sensor_msgs/msg/Imu \
    ! fakesink

# Pipeline through videoconvert, queue, and UDP sink
gst-launch-1.0 \
    v4l2src \
    ! videoconvert \
    ! ros2attach topic=/imu \
    ! queue \
    ! rtpvrawpay \
    ! udpsink host=192.168.1.100 port=5000
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

The plugin installs to the system GStreamer plugin directory (detected via `pkg-config`), so `gst-inspect-1.0 ros2gstmeta` works without setting `GST_PLUGIN_PATH`.

### Dependencies

- ROS 2 Jazzy (`rclcpp`)
- GStreamer 1.0:
  ```bash
  sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
      gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good
  ```
- [gst-metadata](https://github.com/PavelGuzenfeld/gst-metadata) v0.1.0 (fetched automatically via CMake FetchContent)

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

Full end-to-end tests with ROS 2 publishers and GStreamer pipelines (works on both x86_64 and aarch64):

```bash
docker build -t ros2-gst-meta-test .
docker run --rm ros2-gst-meta-test
```

The E2E suite covers:

| Test | What it verifies |
|------|-----------------|
| Plugin load | `gst-inspect-1.0 ros2gstmeta` succeeds |
| Element registration | `ros2attach`, `ros2detach`, `ros2metaprint` all registered |
| Attach pipeline | Pipeline with live ROS 2 publisher reaches EOS |
| videoconvert + videoscale | Metadata survives pixel format and resolution changes |
| queue | Metadata crosses thread boundaries |
| tee | Metadata fans out to multiple branches |
| Deep chain | Metadata survives convert → queue → scale → queue → convert |
| UDP sink | Pipeline with rtpvrawpay → udpsink completes |
| Round-trip | attach → detach publishes correct message (stamp values verified) |
| Multi-topic | Two attach elements on different topics in one pipeline |
| No publisher | Graceful pass-through when topic has no publisher |

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

### Thread safety

- `SyncBuffer` is mutex-protected; `push()` and `pick()` are safe to call from different threads
- `rclcpp::init`/`shutdown` is ref-counted across all elements via `Ros2Lifecycle`
- Spin threads check `rclcpp::ok()` and exit cleanly on SIGINT/SIGTERM
- Each element runs its own background spin thread for ROS 2 callbacks

### Design decisions

- **Fixed-size metadata (4 KB POD)**: Zero heap allocation per buffer. Metadata survives all GStreamer element boundaries (`videoconvert`, `videoscale`, `tee`, `queue`, etc.)
- **Runtime type discovery**: No compile-time message dependencies. The topic type is discovered from the ROS 2 graph at element start
- **FNV-1a topic hashing**: Multiple topics coexist on the same buffer and are distinguished by a 32-bit hash
- **CDR endianness**: `try_extract_stamp` respects the CDR encapsulation endianness flag for cross-platform compatibility
- **Latest mode drain**: In `latest` sync policy, `pick()` keeps only the most recent entry, preventing unbounded memory growth from stale messages

## CI

The GitHub Actions CI pipeline runs on every push and PR:

- **Unit tests** — cmake build + ctest in `ros:jazzy` container
- **Sanitizers** — ASan+UBSan and TSan builds with full test suite
- **E2E** — Docker-based integration tests with live ROS 2 publishers
- **Lint** — cppcheck static analysis

Releases are created automatically on `v*` tags after passing all tests.

## License

MIT — see [LICENSE](LICENSE) for details.
