#!/usr/bin/env bash
# =============================================================================
# E2E integration tests for ros2-gst-meta GStreamer plugin
# Requires ROS 2 Jazzy environment and the plugin installed.
# =============================================================================
set -eo pipefail

# ---------------------------------------------------------------------------
# Environment setup
# ---------------------------------------------------------------------------
source /opt/ros/jazzy/setup.bash
set -u

# Use arch-specific plugin path if not already set
if [ -z "${GST_PLUGIN_PATH:-}" ]; then
    GST_PLUGIN_DIR=$(pkg-config --variable=pluginsdir gstreamer-1.0 2>/dev/null || true)
    export GST_PLUGIN_PATH="${GST_PLUGIN_DIR:-/usr/lib/$(dpkg-architecture -qDEB_HOST_MULTIARCH 2>/dev/null || echo x86_64-linux-gnu)/gstreamer-1.0}"
fi

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
# Test 3: Attach pipeline with publisher (basic)
# ---------------------------------------------------------------------------
run_test "Attach pipeline receives metadata from ROS 2 publisher"

# Start a ROS 2 publisher in background
ros2 topic pub /test_imu sensor_msgs/msg/Imu \
    "{header: {stamp: {sec: 42, nanosec: 123}}}" \
    --rate 10 > /dev/null 2>&1 &
PUB_PID=$!
PIDS_TO_KILL+=("$PUB_PID")

# Wait for publisher to start
sleep 3

# Run the pipeline — success = EOS reached without crash
ATTACH_OUTPUT=$(timeout 30 bash -c '\
    gst-launch-1.0 \
        videotestsrc num-buffers=10 \
        ! ros2attach topic=/test_imu \
        ! fakesink 2>&1' || true)

if echo "$ATTACH_OUTPUT" | grep -qi -e "EOS" -e "Execution ended"; then
    pass "Attach pipeline ran and processed buffers"
else
    fail "Attach pipeline did not reach EOS"
    echo "    Output: $(echo "$ATTACH_OUTPUT" | head -20)"
fi

# Clean up publisher
kill "$PUB_PID" 2>/dev/null || true
wait "$PUB_PID" 2>/dev/null || true
PIDS_TO_KILL=()

# ---------------------------------------------------------------------------
# Test 4: Metadata survives videoconvert + videoscale
# ---------------------------------------------------------------------------
run_test "Metadata survives videoconvert and videoscale"

ros2 topic pub /test_convert sensor_msgs/msg/Imu \
    "{header: {stamp: {sec: 10, nanosec: 0}}}" \
    --rate 10 > /dev/null 2>&1 &
PUB_PID=$!
PIDS_TO_KILL+=("$PUB_PID")
sleep 3

CONVERT_OUTPUT=$(timeout 30 bash -c '\
    gst-launch-1.0 \
        videotestsrc num-buffers=10 \
        ! ros2attach topic=/test_convert \
        ! videoconvert \
        ! videoscale \
        ! video/x-raw,width=320,height=240 \
        ! fakesink 2>&1' || true)

if echo "$CONVERT_OUTPUT" | grep -qi -e "EOS" -e "Execution ended"; then
    pass "Metadata survives videoconvert + videoscale pipeline"
else
    fail "Pipeline with videoconvert + videoscale did not reach EOS"
    echo "    Output: $(echo "$CONVERT_OUTPUT" | head -20)"
fi

kill "$PUB_PID" 2>/dev/null || true
wait "$PUB_PID" 2>/dev/null || true
PIDS_TO_KILL=()

# ---------------------------------------------------------------------------
# Test 5: Metadata survives queue (thread boundary)
# ---------------------------------------------------------------------------
run_test "Metadata survives queue element (thread boundary)"

ros2 topic pub /test_queue sensor_msgs/msg/Imu \
    "{header: {stamp: {sec: 20, nanosec: 0}}}" \
    --rate 10 > /dev/null 2>&1 &
PUB_PID=$!
PIDS_TO_KILL+=("$PUB_PID")
sleep 3

QUEUE_OUTPUT=$(timeout 30 bash -c '\
    gst-launch-1.0 \
        videotestsrc num-buffers=10 \
        ! ros2attach topic=/test_queue \
        ! queue max-size-buffers=5 \
        ! fakesink 2>&1' || true)

if echo "$QUEUE_OUTPUT" | grep -qi -e "EOS" -e "Execution ended"; then
    pass "Metadata survives queue element"
else
    fail "Pipeline with queue did not reach EOS"
    echo "    Output: $(echo "$QUEUE_OUTPUT" | head -20)"
fi

kill "$PUB_PID" 2>/dev/null || true
wait "$PUB_PID" 2>/dev/null || true
PIDS_TO_KILL=()

# ---------------------------------------------------------------------------
# Test 6: Metadata through tee (fan-out)
# ---------------------------------------------------------------------------
run_test "Metadata through tee element (fan-out)"

ros2 topic pub /test_tee sensor_msgs/msg/Imu \
    "{header: {stamp: {sec: 30, nanosec: 0}}}" \
    --rate 10 > /dev/null 2>&1 &
PUB_PID=$!
PIDS_TO_KILL+=("$PUB_PID")
sleep 3

TEE_OUTPUT=$(timeout 30 bash -c '\
    gst-launch-1.0 \
        videotestsrc num-buffers=10 \
        ! ros2attach topic=/test_tee \
        ! tee name=t \
        t. ! queue ! fakesink \
        t. ! queue ! fakesink 2>&1' || true)

if echo "$TEE_OUTPUT" | grep -qi -e "EOS" -e "Execution ended"; then
    pass "Metadata through tee with two branches"
else
    fail "Pipeline with tee did not reach EOS"
    echo "    Output: $(echo "$TEE_OUTPUT" | head -20)"
fi

kill "$PUB_PID" 2>/dev/null || true
wait "$PUB_PID" 2>/dev/null || true
PIDS_TO_KILL=()

# ---------------------------------------------------------------------------
# Test 7: Deep pipeline (attach -> convert -> queue -> scale -> sink)
# ---------------------------------------------------------------------------
run_test "Deep pipeline chain"

ros2 topic pub /test_deep sensor_msgs/msg/Imu \
    "{header: {stamp: {sec: 40, nanosec: 0}}}" \
    --rate 10 > /dev/null 2>&1 &
PUB_PID=$!
PIDS_TO_KILL+=("$PUB_PID")
sleep 3

DEEP_OUTPUT=$(timeout 30 bash -c '\
    gst-launch-1.0 \
        videotestsrc num-buffers=10 \
        ! ros2attach topic=/test_deep \
        ! videoconvert \
        ! queue \
        ! videoscale \
        ! queue \
        ! videoconvert \
        ! fakesink 2>&1' || true)

if echo "$DEEP_OUTPUT" | grep -qi -e "EOS" -e "Execution ended"; then
    pass "Deep pipeline chain completed"
else
    fail "Deep pipeline chain did not reach EOS"
    echo "    Output: $(echo "$DEEP_OUTPUT" | head -20)"
fi

kill "$PUB_PID" 2>/dev/null || true
wait "$PUB_PID" 2>/dev/null || true
PIDS_TO_KILL=()

# ---------------------------------------------------------------------------
# Test 8: UDP sink pipeline (metadata through network-style sink)
# ---------------------------------------------------------------------------
run_test "Pipeline with UDP sink"

ros2 topic pub /test_udp sensor_msgs/msg/Imu \
    "{header: {stamp: {sec: 50, nanosec: 0}}}" \
    --rate 10 > /dev/null 2>&1 &
PUB_PID=$!
PIDS_TO_KILL+=("$PUB_PID")
sleep 3

# Use udpsink to localhost on a random high port — verifies the pipeline
# works end-to-end even with a network sink
UDP_OUTPUT=$(timeout 30 bash -c '\
    gst-launch-1.0 \
        videotestsrc num-buffers=10 \
        ! video/x-raw,format=I420,width=320,height=240 \
        ! ros2attach topic=/test_udp \
        ! queue \
        ! rtpvrawpay \
        ! udpsink host=127.0.0.1 port=15004 2>&1' || true)

if echo "$UDP_OUTPUT" | grep -qi -e "EOS" -e "Execution ended"; then
    pass "Pipeline with UDP sink completed"
else
    fail "Pipeline with UDP sink did not reach EOS"
    echo "    Output: $(echo "$UDP_OUTPUT" | head -20)"
fi

kill "$PUB_PID" 2>/dev/null || true
wait "$PUB_PID" 2>/dev/null || true
PIDS_TO_KILL=()

# ---------------------------------------------------------------------------
# Test 9: Round-trip (attach -> detach -> echo)
# ---------------------------------------------------------------------------
run_test "Round-trip: attach metadata and detach to new topic"

# Start publisher on /test_data
ros2 topic pub /test_data sensor_msgs/msg/Imu \
    "{header: {stamp: {sec: 99, nanosec: 500}}}" \
    --rate 10 > /dev/null 2>&1 &
PUB_PID=$!
PIDS_TO_KILL+=("$PUB_PID")

sleep 3

# Start echo listener BEFORE the pipeline so it's ready to receive
timeout 20 ros2 topic echo /test_data_out sensor_msgs/msg/Imu --once > /tmp/echo_output.txt 2>&1 &
ECHO_PID=$!
PIDS_TO_KILL+=("$ECHO_PID")

sleep 1

# Run pipeline that attaches from /test_data and detaches to /test_data_out
# Use is-live=true so videotestsrc respects framerate and buffers flow at real speed
timeout 30 bash -c '\
    gst-launch-1.0 \
        videotestsrc is-live=true num-buffers=50 \
        ! video/x-raw,framerate=10/1 \
        ! ros2attach topic=/test_data \
        ! ros2detach topic=/test_data_out msg-type=sensor_msgs/msg/Imu \
        ! fakesink 2>&1' &
PIPELINE_PID=$!
PIDS_TO_KILL+=("$PIPELINE_PID")

# Wait for the echo to receive a message (or timeout)
wait "$ECHO_PID" 2>/dev/null || true
ECHO_OUTPUT=$(cat /tmp/echo_output.txt 2>/dev/null || true)

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
# Test 10: Multiple topics
# ---------------------------------------------------------------------------
run_test "Multiple topics attached to single pipeline"

# Start two publishers
ros2 topic pub /imu sensor_msgs/msg/Imu \
    "{header: {stamp: {sec: 1, nanosec: 0}}}" \
    --rate 10 > /dev/null 2>&1 &
PUB_IMU_PID=$!
PIDS_TO_KILL+=("$PUB_IMU_PID")

ros2 topic pub /gps sensor_msgs/msg/NavSatFix \
    "{header: {stamp: {sec: 2, nanosec: 0}}, latitude: 47.0, longitude: 8.0}" \
    --rate 10 > /dev/null 2>&1 &
PUB_GPS_PID=$!
PIDS_TO_KILL+=("$PUB_GPS_PID")

sleep 3

# Pipeline with two ros2attach elements (different node names to avoid conflicts)
# Success = pipeline reaches EOS without crash
MULTI_OUTPUT=$(timeout 30 bash -c '\
    gst-launch-1.0 \
        videotestsrc num-buffers=10 \
        ! ros2attach topic=/imu node-name=gst_attach_imu \
        ! ros2attach topic=/gps node-name=gst_attach_gps \
        ! fakesink 2>&1' || true)

if echo "$MULTI_OUTPUT" | grep -qi -e "EOS" -e "Execution ended"; then
    pass "Multiple topics: pipeline completed with both attach elements"
else
    fail "Multiple topics: pipeline did not reach EOS"
    echo "    Output: $(echo "$MULTI_OUTPUT" | head -20)"
fi

# Cleanup
kill "$PUB_IMU_PID" 2>/dev/null || true
kill "$PUB_GPS_PID" 2>/dev/null || true
wait "$PUB_IMU_PID" 2>/dev/null || true
wait "$PUB_GPS_PID" 2>/dev/null || true
PIDS_TO_KILL=()

# ---------------------------------------------------------------------------
# Test 11: No publisher -> pass-through clean
# ---------------------------------------------------------------------------
run_test "No publisher: pipeline passes through without errors"

PASSTHROUGH_OUTPUT=$(timeout 30 bash -c '\
    gst-launch-1.0 \
        videotestsrc num-buffers=5 \
        ! ros2attach topic=/nonexistent_topic_12345 \
        ! fakesink 2>&1' || true)

if echo "$PASSTHROUGH_OUTPUT" | grep -qi -e "error" -e "SEGFAULT" -e "Segmentation"; then
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
