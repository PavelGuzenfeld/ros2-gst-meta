#include <gst/gst.h>

extern "C" {
GType gst_ros2_attach_get_type(void);
GType gst_ros2_detach_get_type(void);
}

#ifndef PACKAGE
#define PACKAGE "ros2-gst-meta"
#endif

static gboolean plugin_init(GstPlugin* plugin)
{
    gboolean ok = TRUE;
    ok &= gst_element_register(plugin, "ros2attach", GST_RANK_NONE, gst_ros2_attach_get_type());
    ok &= gst_element_register(plugin, "ros2detach", GST_RANK_NONE, gst_ros2_detach_get_type());
    return ok;
}

GST_PLUGIN_DEFINE(GST_VERSION_MAJOR, GST_VERSION_MINOR,
                  ros2gstmeta, "ROS 2 ↔ GStreamer metadata bridge",
                  plugin_init, "0.1", "MIT", "ros2-gst-meta",
                  "https://github.com/PavelGuzenfeld/ros2-gst-meta")
