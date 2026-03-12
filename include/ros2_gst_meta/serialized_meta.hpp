#pragma once

#include <gst-metadata/meta_base.hpp>

#include <cstdint>
#include <cstring>

namespace ros2gstmeta {

/// Maximum serialized CDR payload.  Covers virtually all sensor messages
/// (IMU ~232 B, NavSat ~120 B, PointCloud header ~80 B, Image header ~64 B).
/// Messages exceeding this are dropped with a warning.
static constexpr std::size_t MAX_SERIALIZED_SIZE = 4096;

/// Fixed-size POD carried on every GstBuffer.
struct Ros2MsgData {
    std::uint64_t recv_stamp_ns;                   ///< Monotonic clock when we received the msg
    std::uint64_t msg_stamp_ns;                    ///< Header stamp (0 if the msg has no header)
    std::uint32_t topic_hash;                      ///< FNV-1a hash of topic name
    std::uint32_t serialized_len;                  ///< Actual CDR byte count (≤ MAX_SERIALIZED_SIZE)
    std::uint8_t  serialized[MAX_SERIALIZED_SIZE]; ///< Raw CDR payload
};

// --- GType name strings (must be program-lifetime storage) -----------------
inline constexpr char Ros2MsgApiName[]  = "Ros2MsgMetaAPI";
inline constexpr char Ros2MsgInfoName[] = "Ros2MsgMetaInfo";

/// Metadata type that carries a serialized ROS 2 message on a GstBuffer.
class Ros2MsgMeta
    : public gstmeta::MetaBase<Ros2MsgMeta, Ros2MsgData,
                               Ros2MsgApiName, Ros2MsgInfoName> {
public:
    static constexpr std::uint32_t current_version() { return 1; }
};

// --- Helpers ---------------------------------------------------------------

/// FNV-1a hash (32-bit) of a null-terminated string.
inline constexpr std::uint32_t fnv1a(const char* s) noexcept
{
    std::uint32_t h = 0x811c9dc5u;
    for (; *s != '\0'; ++s) {
        h ^= static_cast<std::uint32_t>(static_cast<unsigned char>(*s));
        h *= 0x01000193u;
    }
    return h;
}

} // namespace ros2gstmeta
