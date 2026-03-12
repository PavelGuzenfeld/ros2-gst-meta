#pragma once

/// Ref-counted rclcpp::init/shutdown shared across all ros2-gst-meta elements.
/// Solves: race on rclcpp::init (#1), missing rclcpp::shutdown (#5).

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <mutex>

namespace ros2gstmeta {

class Ros2Lifecycle {
public:
    static void acquire()
    {
        std::lock_guard<std::mutex> lk(mu());
        if (ref_count()++ == 0) {
            if (!rclcpp::ok()) {
                rclcpp::init(0, nullptr);
            }
        }
    }

    static void release()
    {
        std::lock_guard<std::mutex> lk(mu());
        if (--ref_count() == 0) {
            if (rclcpp::ok()) {
                rclcpp::shutdown();
            }
        }
    }

private:
    static std::mutex& mu()
    {
        static std::mutex m;
        return m;
    }

    static int& ref_count()
    {
        static int count = 0;
        return count;
    }
};

} // namespace ros2gstmeta
