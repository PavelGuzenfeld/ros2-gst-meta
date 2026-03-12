/// GStreamer element: ros2metaprint
///
/// In-place passthrough that reads all Ros2MsgMeta from each buffer and
/// prints a human-readable summary to the GStreamer log (GST_INFO).
/// Does NOT depend on ROS 2 -- only gst-metadata and the local headers.
///
/// Properties:
///   verbose       (bool)    -- print full serialized payload    (default: false)
///   filter-topic  (string)  -- only print meta matching fnv1a  (default: "")

#include <ros2_gst_meta/serialized_meta.hpp>

#include <gst/base/gstbasetransform.h>
#include <gst/gst.h>

#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>

// ---------------------------------------------------------------------------
// GObject boilerplate
// ---------------------------------------------------------------------------

struct GstRos2MetaPrint {
    GstBaseTransform parent;

    // Properties
    gboolean verbose;
    gchar*   filter_topic;

    // Derived from filter_topic
    std::uint32_t filter_hash;
    gboolean      filtering;
};

struct GstRos2MetaPrintClass {
    GstBaseTransformClass parent_class;
};

extern "C" GType gst_ros2_meta_print_get_type(void);
G_DEFINE_TYPE(GstRos2MetaPrint, gst_ros2_meta_print, GST_TYPE_BASE_TRANSFORM)

// ---------------------------------------------------------------------------
// Property IDs
// ---------------------------------------------------------------------------

enum {
    PRINT_PROP_0,
    PRINT_PROP_VERBOSE,
    PRINT_PROP_FILTER_TOPIC,
};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Format up to @a max_bytes of @a data as hex into @a buf (null-terminated).
/// Returns @a buf for convenience.
static char* hex_string(char* buf, std::size_t buf_size,
                        const std::uint8_t* data, std::size_t max_bytes)
{
    std::size_t offset = 0;
    for (std::size_t i = 0; i < max_bytes && (offset + 3) < buf_size; ++i) {
        std::snprintf(buf + offset, buf_size - offset, "%02x ", data[i]);
        offset += 3;
    }
    if (offset > 0 && buf[offset - 1] == ' ') {
        buf[offset - 1] = '\0';
    } else {
        buf[0] = '\0';
    }
    return buf;
}

// ---------------------------------------------------------------------------
// GstBaseTransform vfuncs
// ---------------------------------------------------------------------------

static GstFlowReturn ros2_meta_print_transform_ip(GstBaseTransform* base,
                                                    GstBuffer* buf)
{
    auto* self = reinterpret_cast<GstRos2MetaPrint*>(base);

    ros2gstmeta::Ros2MsgMeta::for_each(
        buf, [&](const ros2gstmeta::Ros2MsgData& d) {
            // Apply topic filter if set
            if (self->filtering && d.topic_hash != self->filter_hash) {
                return;
            }

            // Always print the summary line
            std::size_t preview_len = d.serialized_len < 16 ? d.serialized_len : 16;
            char hex_buf[16 * 3 + 1];
            hex_string(hex_buf, sizeof(hex_buf), d.serialized, preview_len);

            GST_INFO_OBJECT(self,
                "Ros2Meta: topic_hash=0x%08" PRIx32
                " recv_stamp=%" PRIu64
                " msg_stamp=%" PRIu64
                " serialized_len=%" PRIu32
                " data=[%s]",
                d.topic_hash,
                d.recv_stamp_ns,
                d.msg_stamp_ns,
                d.serialized_len,
                hex_buf);

            // In verbose mode, dump the full payload (heap-allocated for safety)
            if (self->verbose && d.serialized_len > 16) {
                const std::size_t full_buf_size = d.serialized_len * 3 + 1;
                auto full_buf = std::make_unique<char[]>(full_buf_size);
                hex_string(full_buf.get(), full_buf_size,
                           d.serialized, d.serialized_len);
                GST_INFO_OBJECT(self, "  full payload: [%s]", full_buf.get());
            }
        });

    return GST_FLOW_OK;
}

// ---------------------------------------------------------------------------
// GObject property accessors
// ---------------------------------------------------------------------------

static void gst_ros2_meta_print_set_property(GObject* object, guint prop_id,
                                              const GValue* value,
                                              [[maybe_unused]] GParamSpec* pspec)
{
    auto* self = reinterpret_cast<GstRos2MetaPrint*>(object);
    switch (prop_id) {
    case PRINT_PROP_VERBOSE:
        self->verbose = g_value_get_boolean(value);
        break;
    case PRINT_PROP_FILTER_TOPIC:
        g_free(self->filter_topic);
        self->filter_topic = g_value_dup_string(value);
        if (self->filter_topic && self->filter_topic[0] != '\0') {
            self->filter_hash = ros2gstmeta::fnv1a(self->filter_topic);
            self->filtering = TRUE;
        } else {
            self->filter_hash = 0;
            self->filtering = FALSE;
        }
        break;
    default:
        G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
        break;
    }
}

static void gst_ros2_meta_print_get_property(GObject* object, guint prop_id,
                                              GValue* value,
                                              [[maybe_unused]] GParamSpec* pspec)
{
    auto* self = reinterpret_cast<GstRos2MetaPrint*>(object);
    switch (prop_id) {
    case PRINT_PROP_VERBOSE:
        g_value_set_boolean(value, self->verbose);
        break;
    case PRINT_PROP_FILTER_TOPIC:
        g_value_set_string(value, self->filter_topic);
        break;
    default:
        G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
        break;
    }
}

static void gst_ros2_meta_print_finalize(GObject* object)
{
    auto* self = reinterpret_cast<GstRos2MetaPrint*>(object);
    g_free(self->filter_topic);
    G_OBJECT_CLASS(gst_ros2_meta_print_parent_class)->finalize(object);
}

// ---------------------------------------------------------------------------
// Class / instance init
// ---------------------------------------------------------------------------

static GstStaticPadTemplate print_sink_tmpl = GST_STATIC_PAD_TEMPLATE(
    "sink", GST_PAD_SINK, GST_PAD_ALWAYS, GST_STATIC_CAPS_ANY);
static GstStaticPadTemplate print_src_tmpl = GST_STATIC_PAD_TEMPLATE(
    "src", GST_PAD_SRC, GST_PAD_ALWAYS, GST_STATIC_CAPS_ANY);

static void gst_ros2_meta_print_class_init(GstRos2MetaPrintClass* klass)
{
    auto* gobject_class = G_OBJECT_CLASS(klass);
    gobject_class->set_property = gst_ros2_meta_print_set_property;
    gobject_class->get_property = gst_ros2_meta_print_get_property;
    gobject_class->finalize     = gst_ros2_meta_print_finalize;

    g_object_class_install_property(
        gobject_class, PRINT_PROP_VERBOSE,
        g_param_spec_boolean("verbose", "Verbose",
                             "Print all serialized bytes (not just first 16)",
                             FALSE,
                             static_cast<GParamFlags>(G_PARAM_READWRITE |
                                                      G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(
        gobject_class, PRINT_PROP_FILTER_TOPIC,
        g_param_spec_string("filter-topic", "Filter Topic",
                            "Only print metadata matching this topic's fnv1a hash "
                            "(empty = print all)",
                            "",
                            static_cast<GParamFlags>(G_PARAM_READWRITE |
                                                     G_PARAM_STATIC_STRINGS)));

    auto* element_class = GST_ELEMENT_CLASS(klass);
    gst_element_class_set_static_metadata(element_class,
        "ROS 2 Metadata Print", "Debug/Metadata",
        "Prints Ros2MsgMeta attached to each buffer",
        "ros2-gst-meta");
    gst_element_class_add_static_pad_template(element_class, &print_sink_tmpl);
    gst_element_class_add_static_pad_template(element_class, &print_src_tmpl);

    auto* bt = GST_BASE_TRANSFORM_CLASS(klass);
    bt->transform_ip = ros2_meta_print_transform_ip;
}

static void gst_ros2_meta_print_init(GstRos2MetaPrint* self)
{
    gst_base_transform_set_in_place(GST_BASE_TRANSFORM(self), TRUE);
    gst_base_transform_set_passthrough(GST_BASE_TRANSFORM(self), TRUE);
    self->verbose      = FALSE;
    self->filter_topic = g_strdup("");
    self->filter_hash  = 0;
    self->filtering    = FALSE;
}
