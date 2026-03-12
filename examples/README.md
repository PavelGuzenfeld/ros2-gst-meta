# ros2-gst-meta Examples

## Prerequisites

1. Source a ROS 2 Jazzy workspace.
2. Build the project with colcon:

```bash
colcon build --packages-select ros2_gst_meta
source install/setup.bash
```

3. Point GStreamer at the plugin:

```bash
export GST_PLUGIN_PATH=$COLCON_PREFIX_PATH/lib
```

4. Verify the plugin is visible:

```bash
gst-inspect-1.0 ros2gstmeta
```

You should see `ros2attach`, `ros2detach`, and `ros2metaprint` listed.

## Demo Pipelines

All demos are in `pipeline_demo.sh`. Run without arguments to see help:

```bash
./examples/pipeline_demo.sh
```

### Example 1 -- Attach and Print

Attaches `/imu` metadata to a test video stream and prints each buffer's metadata.

Terminal 1 (publisher):
```bash
ros2 topic pub /imu sensor_msgs/msg/Imu \
    "{header: {stamp: {sec: 1, nanosec: 0}}}" --rate 30
```

Terminal 2 (pipeline):
```bash
./examples/pipeline_demo.sh example1
```

Expected output (via GST_DEBUG):
```
ros2metaprint: Ros2Meta: topic_hash=0x1a2b3c4d recv_stamp=1234567890 msg_stamp=1000000000 serialized_len=232 data=[00 01 00 00 01 00 00 00 ...]
```

### Example 2 -- Multiple Topics

Attaches both `/imu` and `/navsat` metadata and prints all with `verbose=true`
(full serialized payload hex dump).

```bash
./examples/pipeline_demo.sh example2
```

### Example 3 -- Round-trip (Attach, Print, Detach)

Attaches `/imu`, prints only that topic's metadata using `filter-topic=/imu`,
then republishes the serialized CDR back to `/imu_out`.

Terminal 1 (publisher):
```bash
ros2 topic pub /imu sensor_msgs/msg/Imu \
    "{header: {stamp: {sec: 1, nanosec: 0}}}" --rate 30
```

Terminal 2 (pipeline):
```bash
./examples/pipeline_demo.sh example3
```

Terminal 3 (listener):
```bash
ros2 topic echo /imu_out sensor_msgs/msg/Imu
```

## Element Properties

### ros2metaprint

| Property       | Type    | Default | Description                                      |
|----------------|---------|---------|--------------------------------------------------|
| `verbose`      | boolean | false   | Print all serialized bytes (not just first 16)   |
| `filter-topic` | string  | ""      | Only print meta matching this topic's fnv1a hash |

Set `GST_DEBUG=ros2metaprint:5` to see the output (it uses `GST_INFO_OBJECT`).
