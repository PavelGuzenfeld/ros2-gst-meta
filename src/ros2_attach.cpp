/// GStreamer element: ros2attach
///
/// Subscribes to a ROS 2 topic (any message type, discovered at runtime),
/// and attaches the serialized CDR payload as Ros2MsgMeta on every buffer
/// that flows through.
///
/// Properties:
///   topic        (string)  — ROS 2 topic name          (default: "")
///   sync-policy  (enum)    — latest | nearest           (default: latest)
///   max-age-ms   (uint)    — max staleness in ms, 0=∞   (default: 0)
///   node-name    (string)  — ROS 2 node name            (default: "gst_ros2_attach")

#include <ros2_gst_meta/serialized_meta.hpp>
#include <ros2_gst_meta/sync_buffer.hpp>

#include <gst/base/gstbasetransform.h>
#include <gst/gst.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/generic_subscription.hpp>

#include <chrono>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

// ---------------------------------------------------------------------------
// GObject boilerplate
// ---------------------------------------------------------------------------

struct GstRos2Attach {
    GstBaseTransform parent;

    // Properties
    gchar* topic;
    gchar* node_name;
    guint  max_age_ms;
    gint   sync_policy;  // 0=latest, 1=nearest

    // Runtime state
    std::shared_ptr<rclcpp::Node>                node;
    std::shared_ptr<rclcpp::GenericSubscription>  sub;
    std::unique_ptr<ros2gstmeta::SyncBuffer>     sync_buf;
    std::thread                                   spin_thread;
    std::atomic<bool>                             spinning{false};
    std::uint32_t                                 topic_hash{0};
};

struct GstRos2AttachClass {
    GstBaseTransformClass parent_class;
};

extern "C" GType gst_ros2_attach_get_type(void);
G_DEFINE_TYPE(GstRos2Attach, gst_ros2_attach, GST_TYPE_BASE_TRANSFORM)

// ---------------------------------------------------------------------------
// Property IDs and enum registration
// ---------------------------------------------------------------------------

enum {
    PROP_0,
    PROP_TOPIC,
    PROP_SYNC_POLICY,
    PROP_MAX_AGE_MS,
    PROP_NODE_NAME,
};

#define GST_TYPE_ROS2_SYNC_POLICY (gst_ros2_sync_policy_get_type())

static GType gst_ros2_sync_policy_get_type()
{
    static GType type = 0;
    if (g_once_init_enter(&type)) {
        static const GEnumValue values[] = {
            {0, "Always use most recent message", "latest"},
            {1, "Closest timestamp match", "nearest"},
            {0, nullptr, nullptr},
        };
        GType t = g_enum_register_static("GstRos2SyncPolicy", values);
        g_once_init_leave(&type, t);
    }
    return type;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static std::uint64_t monotonic_now_ns()
{
    auto now = std::chrono::steady_clock::now();
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            now.time_since_epoch())
            .count());
}

/// Try to extract header.stamp from the first 8 bytes of CDR data.
/// ROS 2 messages with a std_msgs/Header always start with the stamp
/// (after CDR encapsulation header).  Returns 0 on failure.
static std::uint64_t try_extract_stamp([[maybe_unused]] const uint8_t* cdr,
                                        [[maybe_unused]] std::size_t len)
{
    // CDR encapsulation: 4 bytes header, then the message fields.
    // For messages with a std_msgs/Header, the stamp is:
    //   offset 4: int32 sec
    //   offset 8: uint32 nanosec
    // This is a best-effort heuristic; returns 0 if data is too short.
    if (len < 12) return 0;

    std::int32_t sec = 0;
    std::uint32_t nsec = 0;
    std::memcpy(&sec, cdr + 4, sizeof(sec));
    std::memcpy(&nsec, cdr + 8, sizeof(nsec));

    if (sec < 0) return 0;
    return static_cast<std::uint64_t>(sec) * 1'000'000'000ULL + nsec;
}

// ---------------------------------------------------------------------------
// ROS 2 lifecycle
// ---------------------------------------------------------------------------

static void ros2_start(GstRos2Attach* self)
{
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }

    const char* node_name = (self->node_name && self->node_name[0])
                                ? self->node_name
                                : "gst_ros2_attach";

    self->node = std::make_shared<rclcpp::Node>(node_name);

    auto policy = (self->sync_policy == 1) ? ros2gstmeta::SyncBuffer::Policy::Nearest
                                           : ros2gstmeta::SyncBuffer::Policy::Latest;
    std::uint64_t max_age_ns = static_cast<std::uint64_t>(self->max_age_ms) * 1'000'000ULL;
    self->sync_buf = std::make_unique<ros2gstmeta::SyncBuffer>(policy, max_age_ns);
    self->topic_hash = ros2gstmeta::fnv1a(self->topic);

    // Discover topic type from the ROS graph
    std::string topic_type;
    for (int attempt = 0; attempt < 50; ++attempt) {
        auto topics = self->node->get_topic_names_and_types();
        auto it = topics.find(std::string(self->topic));
        if (it != topics.end() && !it->second.empty()) {
            topic_type = it->second.front();
            break;
        }
        rclcpp::spin_some(self->node);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (topic_type.empty()) {
        GST_ERROR_OBJECT(self, "Could not discover type for topic '%s' after 5 s", self->topic);
        return;
    }

    GST_INFO_OBJECT(self, "Subscribing to %s [%s]", self->topic, topic_type.c_str());

    auto qos = rclcpp::SensorDataQoS();

    self->sub = self->node->create_generic_subscription(
        std::string(self->topic),
        topic_type,
        qos,
        [self](std::shared_ptr<const rclcpp::SerializedMessage> msg) {
            auto& rcl_msg = msg->get_rcl_serialized_message();
            if (rcl_msg.buffer_length > ros2gstmeta::MAX_SERIALIZED_SIZE) {
                GST_WARNING_OBJECT(self,
                    "Message too large (%zu > %zu), dropping",
                    rcl_msg.buffer_length, ros2gstmeta::MAX_SERIALIZED_SIZE);
                return;
            }

            ros2gstmeta::StampedBlob blob;
            blob.recv_stamp_ns = monotonic_now_ns();
            blob.cdr.assign(rcl_msg.buffer,
                            rcl_msg.buffer + rcl_msg.buffer_length);
            blob.msg_stamp_ns = try_extract_stamp(
                rcl_msg.buffer, rcl_msg.buffer_length);

            self->sync_buf->push(std::move(blob));
        });

    // Spin in background thread
    self->spinning = true;
    self->spin_thread = std::thread([self]() {
        while (self->spinning.load(std::memory_order_relaxed)) {
            rclcpp::spin_some(self->node);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
}

static void ros2_stop(GstRos2Attach* self)
{
    self->spinning = false;
    if (self->spin_thread.joinable()) {
        self->spin_thread.join();
    }
    self->sub.reset();
    self->node.reset();
    self->sync_buf.reset();
}

// ---------------------------------------------------------------------------
// GstBaseTransform vfuncs
// ---------------------------------------------------------------------------

static gboolean ros2_attach_start(GstBaseTransform* base)
{
    auto* self = reinterpret_cast<GstRos2Attach*>(base);
    if (!self->topic || self->topic[0] == '\0') {
        GST_ERROR_OBJECT(self, "No topic set — set the 'topic' property");
        return FALSE;
    }
    ros2_start(self);
    return TRUE;
}

static gboolean ros2_attach_stop(GstBaseTransform* base)
{
    ros2_stop(reinterpret_cast<GstRos2Attach*>(base));
    return TRUE;
}

static GstFlowReturn ros2_attach_transform_ip(GstBaseTransform* base,
                                                GstBuffer* buf)
{
    auto* self = reinterpret_cast<GstRos2Attach*>(base);
    if (!self->sync_buf) return GST_FLOW_OK;

    std::uint64_t query_ns = GST_BUFFER_PTS_IS_VALID(buf)
                                 ? GST_BUFFER_PTS(buf)
                                 : monotonic_now_ns();

    auto blob = self->sync_buf->pick(query_ns, monotonic_now_ns());
    if (!blob) return GST_FLOW_OK;  // no message yet — pass through clean

    ros2gstmeta::Ros2MsgData meta_data{};
    meta_data.recv_stamp_ns = blob->recv_stamp_ns;
    meta_data.msg_stamp_ns  = blob->msg_stamp_ns;
    meta_data.topic_hash    = self->topic_hash;
    meta_data.serialized_len = static_cast<std::uint32_t>(blob->cdr.size());
    std::memcpy(meta_data.serialized, blob->cdr.data(), blob->cdr.size());

    ros2gstmeta::Ros2MsgMeta::add(buf, meta_data);
    return GST_FLOW_OK;
}

// ---------------------------------------------------------------------------
// GObject property accessors
// ---------------------------------------------------------------------------

static void gst_ros2_attach_set_property(GObject* object, guint prop_id,
                                          const GValue* value,
                                          [[maybe_unused]] GParamSpec* pspec)
{
    auto* self = reinterpret_cast<GstRos2Attach*>(object);
    switch (prop_id) {
    case PROP_TOPIC:
        g_free(self->topic);
        self->topic = g_value_dup_string(value);
        break;
    case PROP_SYNC_POLICY:
        self->sync_policy = g_value_get_enum(value);
        break;
    case PROP_MAX_AGE_MS:
        self->max_age_ms = g_value_get_uint(value);
        break;
    case PROP_NODE_NAME:
        g_free(self->node_name);
        self->node_name = g_value_dup_string(value);
        break;
    default:
        G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
        break;
    }
}

static void gst_ros2_attach_get_property(GObject* object, guint prop_id,
                                          GValue* value,
                                          [[maybe_unused]] GParamSpec* pspec)
{
    auto* self = reinterpret_cast<GstRos2Attach*>(object);
    switch (prop_id) {
    case PROP_TOPIC:
        g_value_set_string(value, self->topic);
        break;
    case PROP_SYNC_POLICY:
        g_value_set_enum(value, self->sync_policy);
        break;
    case PROP_MAX_AGE_MS:
        g_value_set_uint(value, self->max_age_ms);
        break;
    case PROP_NODE_NAME:
        g_value_set_string(value, self->node_name);
        break;
    default:
        G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
        break;
    }
}

static void gst_ros2_attach_finalize(GObject* object)
{
    auto* self = reinterpret_cast<GstRos2Attach*>(object);
    ros2_stop(self);
    g_free(self->topic);
    g_free(self->node_name);
    G_OBJECT_CLASS(gst_ros2_attach_parent_class)->finalize(object);
}

// ---------------------------------------------------------------------------
// Class / instance init
// ---------------------------------------------------------------------------

static GstStaticPadTemplate sink_tmpl = GST_STATIC_PAD_TEMPLATE(
    "sink", GST_PAD_SINK, GST_PAD_ALWAYS, GST_STATIC_CAPS_ANY);
static GstStaticPadTemplate src_tmpl = GST_STATIC_PAD_TEMPLATE(
    "src", GST_PAD_SRC, GST_PAD_ALWAYS, GST_STATIC_CAPS_ANY);

static void gst_ros2_attach_class_init(GstRos2AttachClass* klass)
{
    auto* gobject_class = G_OBJECT_CLASS(klass);
    gobject_class->set_property = gst_ros2_attach_set_property;
    gobject_class->get_property = gst_ros2_attach_get_property;
    gobject_class->finalize     = gst_ros2_attach_finalize;

    g_object_class_install_property(
        gobject_class, PROP_TOPIC,
        g_param_spec_string("topic", "Topic",
                            "ROS 2 topic to subscribe to",
                            "", static_cast<GParamFlags>(G_PARAM_READWRITE |
                                                         G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(
        gobject_class, PROP_SYNC_POLICY,
        g_param_spec_enum("sync-policy", "Sync Policy",
                          "How to match ROS 2 messages to GStreamer buffers",
                          GST_TYPE_ROS2_SYNC_POLICY, 0,
                          static_cast<GParamFlags>(G_PARAM_READWRITE |
                                                   G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(
        gobject_class, PROP_MAX_AGE_MS,
        g_param_spec_uint("max-age-ms", "Max Age (ms)",
                          "Discard messages older than this (0 = no limit)",
                          0, G_MAXUINT, 0,
                          static_cast<GParamFlags>(G_PARAM_READWRITE |
                                                   G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(
        gobject_class, PROP_NODE_NAME,
        g_param_spec_string("node-name", "Node Name",
                            "ROS 2 node name",
                            "gst_ros2_attach",
                            static_cast<GParamFlags>(G_PARAM_READWRITE |
                                                     G_PARAM_STATIC_STRINGS)));

    auto* element_class = GST_ELEMENT_CLASS(klass);
    gst_element_class_set_static_metadata(element_class,
        "ROS 2 Metadata Attach", "Filter/Metadata",
        "Subscribes to a ROS 2 topic and attaches serialized messages as buffer metadata",
        "ros2-gst-meta");
    gst_element_class_add_static_pad_template(element_class, &sink_tmpl);
    gst_element_class_add_static_pad_template(element_class, &src_tmpl);

    auto* bt = GST_BASE_TRANSFORM_CLASS(klass);
    bt->start        = ros2_attach_start;
    bt->stop         = ros2_attach_stop;
    bt->transform_ip = ros2_attach_transform_ip;
}

static void gst_ros2_attach_init(GstRos2Attach* self)
{
    gst_base_transform_set_in_place(GST_BASE_TRANSFORM(self), TRUE);
    gst_base_transform_set_passthrough(GST_BASE_TRANSFORM(self), TRUE);
    self->topic       = g_strdup("");
    self->node_name   = g_strdup("gst_ros2_attach");
    self->max_age_ms  = 0;
    self->sync_policy = 0;
}
