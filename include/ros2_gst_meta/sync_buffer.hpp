#pragma once

#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <vector>

namespace ros2gstmeta {

/// A timestamped entry stored in the sync buffer.
struct StampedBlob {
    std::uint64_t recv_stamp_ns;    ///< Monotonic receive time
    std::uint64_t msg_stamp_ns;     ///< Message header stamp (0 if none)
    std::vector<std::uint8_t> cdr;  ///< Serialized CDR bytes
};

/// Thread-safe buffer that pairs incoming ROS 2 messages with GStreamer buffer
/// timestamps according to a configurable sync policy.
///
/// The ROS 2 callback thread calls push(); the GStreamer streaming thread
/// calls pick().
class SyncBuffer {
public:
    enum class Policy {
        Latest,   ///< Always return the most recently received message
        Nearest,  ///< Return the message whose msg_stamp is closest to query_ts
    };

    /// @param policy        Sync strategy
    /// @param max_age_ns    Messages older than this are pruned (0 = unlimited)
    /// @param capacity      Max entries kept in the deque (only for Nearest)
    explicit SyncBuffer(Policy policy = Policy::Latest,
                        std::uint64_t max_age_ns = 0,
                        std::size_t capacity = 256)
        : policy_(policy), max_age_ns_(max_age_ns), capacity_(capacity)
    {
    }

    /// Called from the ROS 2 subscription callback thread.
    void push(StampedBlob blob)
    {
        std::lock_guard<std::mutex> lk(mu_);
        buf_.push_back(std::move(blob));
        if (buf_.size() > capacity_) {
            buf_.pop_front();
        }
    }

    /// Called from the GStreamer streaming thread.
    /// @param query_ns  Buffer PTS converted to nanoseconds (used by Nearest).
    /// @param now_ns    Current monotonic time (used for staleness pruning).
    std::optional<StampedBlob> pick(std::uint64_t query_ns,
                                    std::uint64_t now_ns)
    {
        std::lock_guard<std::mutex> lk(mu_);
        prune_locked(now_ns);

        if (buf_.empty()) {
            return std::nullopt;
        }

        switch (policy_) {
        case Policy::Latest: {
            // Keep only the latest entry, drain all stale ones
            if (buf_.size() > 1) {
                auto latest = std::move(buf_.back());
                buf_.clear();
                buf_.push_back(std::move(latest));
            }
            return buf_.back();
        }

        case Policy::Nearest:
            return find_nearest_locked(query_ns);
        }
        return std::nullopt;  // unreachable
    }

    /// Number of buffered entries (for diagnostics).
    std::size_t size() const
    {
        std::lock_guard<std::mutex> lk(mu_);
        return buf_.size();
    }

    void set_policy(Policy p)
    {
        std::lock_guard<std::mutex> lk(mu_);
        policy_ = p;
    }

    void set_max_age(std::uint64_t ns)
    {
        std::lock_guard<std::mutex> lk(mu_);
        max_age_ns_ = ns;
    }

private:
    void prune_locked(std::uint64_t now_ns)
    {
        if (max_age_ns_ == 0) return;
        while (!buf_.empty()) {
            if (now_ns > buf_.front().recv_stamp_ns &&
                (now_ns - buf_.front().recv_stamp_ns) > max_age_ns_) {
                buf_.pop_front();
            } else {
                break;
            }
        }
    }

    StampedBlob find_nearest_locked(std::uint64_t query_ns) const
    {
        auto best = buf_.begin();
        std::uint64_t best_dist = ts_distance(best->msg_stamp_ns, query_ns);

        for (auto it = std::next(buf_.begin()); it != buf_.end(); ++it) {
            std::uint64_t d = ts_distance(it->msg_stamp_ns, query_ns);
            if (d < best_dist) {
                best_dist = d;
                best = it;
            }
        }
        return *best;
    }

    static std::uint64_t ts_distance(std::uint64_t a, std::uint64_t b)
    {
        return a > b ? a - b : b - a;
    }

    mutable std::mutex mu_;
    Policy policy_;
    std::uint64_t max_age_ns_;
    std::size_t capacity_;
    std::deque<StampedBlob> buf_;
};

} // namespace ros2gstmeta
