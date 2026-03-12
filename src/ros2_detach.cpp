/// GStreamer element: ros2detach
///
/// Reads Ros2MsgMeta from buffers and publishes the serialized CDR payload
/// back to a ROS 2 topic.  Useful for bridging GStreamer pipelines back
/// into the ROS 2 graph, or for recording metadata alongside video.
///
/// Properties:
///   topic        (string)  — ROS 2 topic to publish to   (default: "")
///   msg-type     (string)  — ROS 2 message type string   (default: "")
///   node-name    (string)  — ROS 2 node name             (default: "gst_ros2_detach")
///   filter-topic (string)  — only publish from this source topic (default: "" = all)
///   qos-profile  (enum)    — sensor | reliable | system   (default: sensor)

#include <ros2_gst_meta/ros2_lifecycle.hpp>
#include <ros2_gst_meta/serialized_meta.hpp>

#include <gst/base/gstbasetransform.h>
#include <gst/gst.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/serialized_message.hpp>

#include <cstring>
#include <memory>
#include <string>
#include <thread>

// ---------------------------------------------------------------------------
// QoS profile enum (shared with ros2_attach via identical registration)
// ---------------------------------------------------------------------------

#define GST_TYPE_ROS2_DETACH_QOS_PROFILE (gst_ros2_detach_qos_profile_get_type())

static GType gst_ros2_detach_qos_profile_get_type()
{
    static GType type = 0;
    if (g_once_init_enter(&type)) {
        static const GEnumValue values[] = {
            {1, "Best-effort, volatile, keep-last(5)", "sensor"},
            {2, "Reliable, volatile, keep-last(10)", "reliable"},
            {3, "System default", "system-default"},
            {0, nullptr, nullptr},
        };
        GType t = g_enum_register_static("GstRos2DetachQosProfile", values);
        g_once_init_leave(&type, t);
    }
    return type;
}

static rclcpp::QoS qos_from_profile(gint profile)
{
    switch (profile) {
    case 2: return rclcpp::QoS(10).reliable();
    case 3: return rclcpp::SystemDefaultsQoS();
    default: return rclcpp::SensorDataQoS();
    }
}

// ---------------------------------------------------------------------------
// GObject boilerplate
// ---------------------------------------------------------------------------

struct GstRos2Detach {
    GstBaseTransform parent;

    // Properties
    gchar* topic;
    gchar* msg_type;
    gchar* node_name;
    gchar* filter_topic;
    gint   qos_profile;  // 1=sensor, 2=reliable, 3=system_default

    // Runtime state
    std::shared_ptr<rclcpp::Node>             node;
    std::shared_ptr<rclcpp::GenericPublisher> pub;
    std::thread                               spin_thread;
    std::atomic<bool>                         spinning{false};
    std::uint32_t                             filter_hash{0};
    gboolean                                  filtering{FALSE};
    gboolean                                  started{FALSE};
};

struct GstRos2DetachClass {
    GstBaseTransformClass parent_class;
};

extern "C" GType gst_ros2_detach_get_type(void);
G_DEFINE_TYPE(GstRos2Detach, gst_ros2_detach, GST_TYPE_BASE_TRANSFORM)

enum {
    DETACH_PROP_0,
    DETACH_PROP_TOPIC,
    DETACH_PROP_MSG_TYPE,
    DETACH_PROP_NODE_NAME,
    DETACH_PROP_FILTER_TOPIC,
    DETACH_PROP_QOS_PROFILE,
};

// ---------------------------------------------------------------------------
// ROS 2 lifecycle
// ---------------------------------------------------------------------------

static void detach_ros2_start(GstRos2Detach* self)
{
    // Issue #1: Thread-safe ref-counted init
    ros2gstmeta::Ros2Lifecycle::acquire();

    const char* node_name = (self->node_name && self->node_name[0])
                                ? self->node_name
                                : "gst_ros2_detach";

    self->node = std::make_shared<rclcpp::Node>(node_name);

    if (self->filter_topic && self->filter_topic[0] != '\0') {
        self->filter_hash = ros2gstmeta::fnv1a(self->filter_topic);
        self->filtering = TRUE;
    } else {
        self->filter_hash = 0;
        self->filtering = FALSE;
    }

    // Issue #9: Configurable QoS
    auto qos = qos_from_profile(self->qos_profile);
    self->pub = self->node->create_generic_publisher(
        std::string(self->topic),
        std::string(self->msg_type),
        qos);

    GST_INFO_OBJECT(self, "Publishing to %s [%s]", self->topic, self->msg_type);

    // Issue #6 & #17: Check rclcpp::ok() in spin loop
    self->spinning = true;
    self->spin_thread = std::thread([self]() {
        while (self->spinning.load(std::memory_order_relaxed) && rclcpp::ok()) {
            rclcpp::spin_some(self->node);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    self->started = TRUE;
}

static void detach_ros2_stop(GstRos2Detach* self)
{
    self->spinning = false;
    if (self->spin_thread.joinable()) {
        self->spin_thread.join();
    }
    self->pub.reset();
    self->node.reset();

    // Issue #5: Ref-counted shutdown
    if (self->started) {
        ros2gstmeta::Ros2Lifecycle::release();
        self->started = FALSE;
    }
}

// ---------------------------------------------------------------------------
// GstBaseTransform vfuncs
// ---------------------------------------------------------------------------

static gboolean ros2_detach_start(GstBaseTransform* base)
{
    auto* self = reinterpret_cast<GstRos2Detach*>(base);
    if (!self->topic || self->topic[0] == '\0') {
        GST_ERROR_OBJECT(self, "No topic set");
        return FALSE;
    }
    if (!self->msg_type || self->msg_type[0] == '\0') {
        GST_ERROR_OBJECT(self, "No msg-type set");
        return FALSE;
    }
    detach_ros2_start(self);
    return TRUE;
}

static gboolean ros2_detach_stop(GstBaseTransform* base)
{
    detach_ros2_stop(reinterpret_cast<GstRos2Detach*>(base));
    return TRUE;
}

static GstFlowReturn ros2_detach_transform_ip(GstBaseTransform* base,
                                                GstBuffer* buf)
{
    auto* self = reinterpret_cast<GstRos2Detach*>(base);
    if (!self->pub) return GST_FLOW_OK;

    ros2gstmeta::Ros2MsgMeta::for_each(
        buf, [&](const ros2gstmeta::Ros2MsgData& d) {
            if (self->filtering && d.topic_hash != self->filter_hash) return;
            if (d.serialized_len == 0) return;

            auto serialized = std::make_unique<rclcpp::SerializedMessage>(d.serialized_len);
            auto& rcl_msg = serialized->get_rcl_serialized_message();
            std::memcpy(rcl_msg.buffer, d.serialized, d.serialized_len);
            rcl_msg.buffer_length = d.serialized_len;

            self->pub->publish(*serialized);
        });

    return GST_FLOW_OK;
}

// ---------------------------------------------------------------------------
// GObject property accessors
// ---------------------------------------------------------------------------

static void gst_ros2_detach_set_property(GObject* object, guint prop_id,
                                          const GValue* value,
                                          [[maybe_unused]] GParamSpec* pspec)
{
    auto* self = reinterpret_cast<GstRos2Detach*>(object);
    switch (prop_id) {
    case DETACH_PROP_TOPIC:
        g_free(self->topic);
        self->topic = g_value_dup_string(value);
        break;
    case DETACH_PROP_MSG_TYPE:
        g_free(self->msg_type);
        self->msg_type = g_value_dup_string(value);
        break;
    case DETACH_PROP_NODE_NAME:
        g_free(self->node_name);
        self->node_name = g_value_dup_string(value);
        break;
    case DETACH_PROP_FILTER_TOPIC:
        g_free(self->filter_topic);
        self->filter_topic = g_value_dup_string(value);
        break;
    case DETACH_PROP_QOS_PROFILE:
        self->qos_profile = g_value_get_enum(value);
        break;
    default:
        G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
        break;
    }
}

static void gst_ros2_detach_get_property(GObject* object, guint prop_id,
                                          GValue* value,
                                          [[maybe_unused]] GParamSpec* pspec)
{
    auto* self = reinterpret_cast<GstRos2Detach*>(object);
    switch (prop_id) {
    case DETACH_PROP_TOPIC:
        g_value_set_string(value, self->topic);
        break;
    case DETACH_PROP_MSG_TYPE:
        g_value_set_string(value, self->msg_type);
        break;
    case DETACH_PROP_NODE_NAME:
        g_value_set_string(value, self->node_name);
        break;
    case DETACH_PROP_FILTER_TOPIC:
        g_value_set_string(value, self->filter_topic);
        break;
    case DETACH_PROP_QOS_PROFILE:
        g_value_set_enum(value, self->qos_profile);
        break;
    default:
        G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
        break;
    }
}

static void gst_ros2_detach_finalize(GObject* object)
{
    auto* self = reinterpret_cast<GstRos2Detach*>(object);
    detach_ros2_stop(self);
    g_free(self->topic);
    g_free(self->msg_type);
    g_free(self->node_name);
    g_free(self->filter_topic);
    G_OBJECT_CLASS(gst_ros2_detach_parent_class)->finalize(object);
}

// ---------------------------------------------------------------------------
// Class / instance init
// ---------------------------------------------------------------------------

static GstStaticPadTemplate detach_sink_tmpl = GST_STATIC_PAD_TEMPLATE(
    "sink", GST_PAD_SINK, GST_PAD_ALWAYS, GST_STATIC_CAPS_ANY);
static GstStaticPadTemplate detach_src_tmpl = GST_STATIC_PAD_TEMPLATE(
    "src", GST_PAD_SRC, GST_PAD_ALWAYS, GST_STATIC_CAPS_ANY);

static void gst_ros2_detach_class_init(GstRos2DetachClass* klass)
{
    auto* gobject_class = G_OBJECT_CLASS(klass);
    gobject_class->set_property = gst_ros2_detach_set_property;
    gobject_class->get_property = gst_ros2_detach_get_property;
    gobject_class->finalize     = gst_ros2_detach_finalize;

    g_object_class_install_property(
        gobject_class, DETACH_PROP_TOPIC,
        g_param_spec_string("topic", "Topic",
                            "ROS 2 topic to publish to",
                            "", static_cast<GParamFlags>(G_PARAM_READWRITE |
                                                         G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(
        gobject_class, DETACH_PROP_MSG_TYPE,
        g_param_spec_string("msg-type", "Message Type",
                            "ROS 2 message type (e.g. sensor_msgs/msg/Imu)",
                            "", static_cast<GParamFlags>(G_PARAM_READWRITE |
                                                         G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(
        gobject_class, DETACH_PROP_NODE_NAME,
        g_param_spec_string("node-name", "Node Name",
                            "ROS 2 node name",
                            "gst_ros2_detach",
                            static_cast<GParamFlags>(G_PARAM_READWRITE |
                                                     G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(
        gobject_class, DETACH_PROP_FILTER_TOPIC,
        g_param_spec_string("filter-topic", "Filter Topic",
                            "Only publish metadata from this source topic "
                            "(empty = publish all metadata)",
                            "",
                            static_cast<GParamFlags>(G_PARAM_READWRITE |
                                                     G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(
        gobject_class, DETACH_PROP_QOS_PROFILE,
        g_param_spec_enum("qos-profile", "QoS Profile",
                          "ROS 2 QoS profile for the publisher",
                          GST_TYPE_ROS2_DETACH_QOS_PROFILE, 1,  // default=sensor(1)
                          static_cast<GParamFlags>(G_PARAM_READWRITE |
                                                   G_PARAM_STATIC_STRINGS)));

    auto* element_class = GST_ELEMENT_CLASS(klass);
    gst_element_class_set_static_metadata(element_class,
        "ROS 2 Metadata Detach", "Filter/Metadata",
        "Reads serialized ROS 2 metadata from buffers and publishes to a ROS 2 topic",
        "ros2-gst-meta");
    gst_element_class_add_static_pad_template(element_class, &detach_sink_tmpl);
    gst_element_class_add_static_pad_template(element_class, &detach_src_tmpl);

    auto* bt = GST_BASE_TRANSFORM_CLASS(klass);
    bt->start        = ros2_detach_start;
    bt->stop         = ros2_detach_stop;
    bt->transform_ip = ros2_detach_transform_ip;
}

static void gst_ros2_detach_init(GstRos2Detach* self)
{
    gst_base_transform_set_in_place(GST_BASE_TRANSFORM(self), TRUE);
    gst_base_transform_set_passthrough(GST_BASE_TRANSFORM(self), TRUE);
    self->topic        = g_strdup("");
    self->msg_type     = g_strdup("");
    self->node_name    = g_strdup("gst_ros2_detach");
    self->filter_topic = g_strdup("");
    self->qos_profile  = 1;  // sensor
    self->started      = FALSE;
}
