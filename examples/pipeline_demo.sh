#!/bin/bash
# Demo pipelines for ros2-gst-meta
# Requires: ROS 2 Jazzy sourced, ros2-gst-meta plugin in GST_PLUGIN_PATH
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# --------------------------------------------------------------------------
# Prerequisites
# --------------------------------------------------------------------------

print_usage() {
    cat <<'USAGE'
Usage: pipeline_demo.sh [example1 | example2 | example3 | inspect]

  example1  - Attach IMU metadata to a test video and print it
  example2  - Attach multiple topics (IMU + NavSat) and print all
  example3  - Round-trip: attach -> print -> detach
  inspect   - List the ros2-gst-meta plugin elements

Environment:
  GST_PLUGIN_PATH  Must include the directory containing libgstros2meta.so
                   e.g.  export GST_PLUGIN_PATH=$COLCON_PREFIX_PATH/lib

  A ROS 2 Jazzy workspace must be sourced before running.
USAGE
}

check_prerequisites() {
    if [ -z "${GST_PLUGIN_PATH:-}" ]; then
        echo "ERROR: GST_PLUGIN_PATH is not set."
        echo "  export GST_PLUGIN_PATH=\$COLCON_PREFIX_PATH/lib"
        exit 1
    fi

    if ! gst-inspect-1.0 ros2attach &>/dev/null; then
        echo "ERROR: ros2attach element not found. Check GST_PLUGIN_PATH."
        exit 1
    fi

    echo "Prerequisites OK."
}

# --------------------------------------------------------------------------
# Example 1: Attach IMU metadata to video and print it
# --------------------------------------------------------------------------
# In another terminal, start an IMU publisher:
#   ros2 topic pub /imu sensor_msgs/msg/Imu \
#       "{header: {stamp: {sec: 1, nanosec: 0}}}" --rate 30

example1() {
    echo "=== Example 1: Attach /imu metadata and print ==="
    echo "Start a publisher first:"
    echo "  ros2 topic pub /imu sensor_msgs/msg/Imu \\"
    echo "      \"{header: {stamp: {sec: 1, nanosec: 0}}}\" --rate 30"
    echo ""
    echo "Running pipeline (Ctrl-C to stop)..."

    GST_DEBUG=ros2metaprint:5 \
    gst-launch-1.0 -v \
        videotestsrc num-buffers=150 ! video/x-raw,framerate=30/1 \
        ! ros2attach topic=/imu \
        ! ros2metaprint \
        ! fakesink sync=true
}

# --------------------------------------------------------------------------
# Example 2: Attach multiple topics
# --------------------------------------------------------------------------
# Start publishers:
#   ros2 topic pub /imu     sensor_msgs/msg/Imu     "{}" --rate 30
#   ros2 topic pub /navsat  sensor_msgs/msg/NavSatFix "{}" --rate 10

example2() {
    echo "=== Example 2: Attach /imu and /navsat, print all ==="
    echo "Start publishers first (see comments in script)."
    echo ""
    echo "Running pipeline (Ctrl-C to stop)..."

    GST_DEBUG=ros2metaprint:5 \
    gst-launch-1.0 -v \
        videotestsrc num-buffers=150 ! video/x-raw,framerate=30/1 \
        ! ros2attach topic=/imu     name=a1 \
        ! ros2attach topic=/navsat  name=a2 \
        ! ros2metaprint verbose=true \
        ! fakesink sync=true
}

# --------------------------------------------------------------------------
# Example 3: Round-trip: attach -> print -> detach
# --------------------------------------------------------------------------
# Attach /imu, print it, then republish to /imu_out

example3() {
    echo "=== Example 3: Round-trip attach -> print -> detach ==="
    echo "Start a publisher first:"
    echo "  ros2 topic pub /imu sensor_msgs/msg/Imu \\"
    echo "      \"{header: {stamp: {sec: 1, nanosec: 0}}}\" --rate 30"
    echo ""
    echo "Metadata will be republished on /imu_out."
    echo "Listen with:  ros2 topic echo /imu_out sensor_msgs/msg/Imu"
    echo ""
    echo "Running pipeline (Ctrl-C to stop)..."

    GST_DEBUG=ros2metaprint:5 \
    gst-launch-1.0 -v \
        videotestsrc num-buffers=150 ! video/x-raw,framerate=30/1 \
        ! ros2attach topic=/imu \
        ! ros2metaprint filter-topic=/imu \
        ! ros2detach topic=/imu_out msg-type=sensor_msgs/msg/Imu \
        ! fakesink sync=true
}

# --------------------------------------------------------------------------
# Inspect: list plugin elements
# --------------------------------------------------------------------------

inspect() {
    echo "=== Inspecting ros2gstmeta plugin ==="
    gst-inspect-1.0 ros2gstmeta
}

# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------

case "${1:-}" in
    example1)  check_prerequisites; example1  ;;
    example2)  check_prerequisites; example2  ;;
    example3)  check_prerequisites; example3  ;;
    inspect)   inspect ;;
    *)         print_usage ;;
esac
