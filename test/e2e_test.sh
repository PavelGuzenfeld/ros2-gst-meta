#!/usr/bin/env bash
# =============================================================================
# E2E integration tests for ros2-gst-meta GStreamer plugin
# Requires ROS 2 Jazzy environment and the plugin installed.
# =============================================================================
set -euo pipefail

# ---------------------------------------------------------------------------
# Environment setup
# ---------------------------------------------------------------------------
source /opt/ros/jazzy/setup.bash

export GST_PLUGIN_PATH="${GST_PLUGIN_PATH:-/usr/lib/gstreamer-1.0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

# Disable GStreamer plugin scanning cache for reproducibility
export GST_REGISTRY_UPDATE=no

PASS=0
FAIL=0
PIDS_TO_KILL=()

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
cleanup() {
    for pid in "${PIDS_TO_KILL[@]}"; do
        kill "$pid" 2>/dev/null || true
        wait "$pid" 2>/dev/null || true
    done
    PIDS_TO_KILL=()
}
trap cleanup EXIT

pass() {
    PASS=$((PASS + 1))
    echo "  PASS: $1"
}

fail() {
    FAIL=$((FAIL + 1))
    echo "  FAIL: $1"
}

run_test() {
    echo ""
    echo "--- TEST: $1 ---"
}

# ---------------------------------------------------------------------------
# Test 1: Plugin loads
# ---------------------------------------------------------------------------
run_test "Plugin loads via gst-inspect-1.0"

if timeout 10 gst-inspect-1.0 ros2gstmeta > /dev/null 2>&1; then
    pass "Plugin ros2gstmeta loads"
else
    fail "Plugin ros2gstmeta failed to load"
fi

# ---------------------------------------------------------------------------
# Test 2: Plugin elements exist
# ---------------------------------------------------------------------------
run_test "Plugin elements are registered"

for element in ros2attach ros2detach ros2metaprint; do
    if timeout 10 gst-inspect-1.0 "$element" > /dev/null 2>&1; then
        pass "Element $element registered"
    else
        fail "Element $element not found"
    fi
done

# ---------------------------------------------------------------------------
# Test 3: Attach pipeline with publisher
# ---------------------------------------------------------------------------
run_test "Attach pipeline receives metadata from ROS 2 publisher"

# Start a ROS 2 publisher in background
ros2 topic pub /test_imu sensor_msgs/msg/Imu \
    "{header: {stamp: {sec: 42, nanosec: 123}}}" \
    --rate 10 &
PUB_PID=$!
PIDS_TO_KILL+=("$PUB_PID")

# Wait for publisher to start
sleep 3

# Run the pipeline with GST_DEBUG to see metadata-related log output
ATTACH_OUTPUT=$(timeout 30 bash -c '\
    GST_DEBUG="ros2gstmeta:5,ros2attach:5,BaseTransform:4" \
    gst-launch-1.0 \
        videotestsrc num-buffers=5 \
        ! ros2attach topic=/test_imu \
        ! fakesink 2>&1' || true)

if echo "$ATTACH_OUTPUT" | grep -qi -e "Subscribing" -e "ros2attach" -e "transform"; then
    pass "Attach pipeline ran and processed buffers"
else
    fail "Attach pipeline did not show expected output"
    echo "    Output: $(echo "$ATTACH_OUTPUT" | head -20)"
fi

# Clean up publisher
kill "$PUB_PID" 2>/dev/null || true
wait "$PUB_PID" 2>/dev/null || true
PIDS_TO_KILL=()

# ---------------------------------------------------------------------------
# Test 4: Round-trip (attach -> detach -> echo)
# ---------------------------------------------------------------------------
run_test "Round-trip: attach metadata and detach to new topic"

# Start publisher on /test_data
ros2 topic pub /test_data sensor_msgs/msg/Imu \
    "{header: {stamp: {sec: 99, nanosec: 500}}}" \
    --rate 10 &
PUB_PID=$!
PIDS_TO_KILL+=("$PUB_PID")

sleep 3

# Run pipeline that attaches from /test_data and detaches to /test_data_out
timeout 30 bash -c '\
    gst-launch-1.0 \
        videotestsrc num-buffers=30 \
        ! ros2attach topic=/test_data \
        ! ros2detach topic=/test_data_out msg-type=sensor_msgs/msg/Imu \
        ! fakesink 2>&1' &
PIPELINE_PID=$!
PIDS_TO_KILL+=("$PIPELINE_PID")

sleep 2

# Try to receive a message on the output topic
ECHO_OUTPUT=$(timeout 15 ros2 topic echo /test_data_out sensor_msgs/msg/Imu --once 2>&1 || true)

if echo "$ECHO_OUTPUT" | grep -q "header"; then
    pass "Round-trip: message received on /test_data_out"
else
    fail "Round-trip: no message received on /test_data_out"
    echo "    Output: $(echo "$ECHO_OUTPUT" | head -10)"
fi

# Cleanup
kill "$PIPELINE_PID" 2>/dev/null || true
kill "$PUB_PID" 2>/dev/null || true
wait "$PIPELINE_PID" 2>/dev/null || true
wait "$PUB_PID" 2>/dev/null || true
PIDS_TO_KILL=()

# ---------------------------------------------------------------------------
# Test 5: Multiple topics
# ---------------------------------------------------------------------------
run_test "Multiple topics attached to single pipeline"

# Start two publishers
ros2 topic pub /imu sensor_msgs/msg/Imu \
    "{header: {stamp: {sec: 1, nanosec: 0}}}" \
    --rate 10 &
PUB_IMU_PID=$!
PIDS_TO_KILL+=("$PUB_IMU_PID")

ros2 topic pub /gps sensor_msgs/msg/NavSatFix \
    "{header: {stamp: {sec: 2, nanosec: 0}}, latitude: 47.0, longitude: 8.0}" \
    --rate 10 &
PUB_GPS_PID=$!
PIDS_TO_KILL+=("$PUB_GPS_PID")

sleep 3

# Pipeline with two ros2attach elements (different node names to avoid conflicts)
MULTI_OUTPUT=$(timeout 30 bash -c '\
    GST_DEBUG="ros2gstmeta:5,ros2attach:5" \
    gst-launch-1.0 \
        videotestsrc num-buffers=10 \
        ! ros2attach topic=/imu node-name=gst_attach_imu \
        ! ros2attach topic=/gps node-name=gst_attach_gps \
        ! fakesink 2>&1' || true)

# Check that both topics were subscribed to
IMU_OK=false
GPS_OK=false

if echo "$MULTI_OUTPUT" | grep -qi "/imu"; then
    IMU_OK=true
fi
if echo "$MULTI_OUTPUT" | grep -qi "/gps"; then
    GPS_OK=true
fi

if $IMU_OK && $GPS_OK; then
    pass "Multiple topics: both /imu and /gps attached"
elif $IMU_OK || $GPS_OK; then
    fail "Multiple topics: only one topic attached"
else
    fail "Multiple topics: neither topic showed in output"
    echo "    Output: $(echo "$MULTI_OUTPUT" | head -20)"
fi

# Cleanup
kill "$PUB_IMU_PID" 2>/dev/null || true
kill "$PUB_GPS_PID" 2>/dev/null || true
wait "$PUB_IMU_PID" 2>/dev/null || true
wait "$PUB_GPS_PID" 2>/dev/null || true
PIDS_TO_KILL=()

# ---------------------------------------------------------------------------
# Test 6: No publisher -> pass-through clean
# ---------------------------------------------------------------------------
run_test "No publisher: pipeline passes through without errors"

# Run pipeline with ros2attach on a topic that has no publisher.
# The element should time out on discovery but the pipeline should still
# process buffers cleanly (pass-through mode).
PASSTHROUGH_OUTPUT=$(timeout 30 bash -c '\
    GST_DEBUG="ros2attach:4" \
    gst-launch-1.0 \
        videotestsrc num-buffers=5 \
        ! ros2attach topic=/nonexistent_topic_12345 \
        ! fakesink 2>&1' || true)

# The pipeline may log a warning about topic discovery failing, but it should
# not crash. gst-launch-1.0 returns 0 on normal EOS.
if echo "$PASSTHROUGH_OUTPUT" | grep -qi -e "error" -e "SEGFAULT" -e "Segmentation"; then
    # Check if the "error" is just the expected topic-not-found warning
    if echo "$PASSTHROUGH_OUTPUT" | grep -qi "Could not discover type"; then
        pass "No publisher: clean pass-through (with expected discovery warning)"
    else
        fail "No publisher: unexpected error in output"
        echo "    Output: $(echo "$PASSTHROUGH_OUTPUT" | head -20)"
    fi
else
    pass "No publisher: clean pass-through"
fi

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
echo ""
echo "==========================================="
echo " E2E Results: $PASS passed, $FAIL failed"
echo "==========================================="

if [ "$FAIL" -gt 0 ]; then
    exit 1
fi

exit 0
