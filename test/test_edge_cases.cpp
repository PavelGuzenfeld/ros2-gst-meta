#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <ros2_gst_meta/serialized_meta.hpp>
#include <ros2_gst_meta/sync_buffer.hpp>

#include <climits>
#include <cstring>
#include <string>

using namespace ros2gstmeta;

namespace {
struct GstInitGuard {
    GstInitGuard() { gst_init(nullptr, nullptr); }
};
static GstInitGuard gst_guard; // NOLINT
} // namespace

// ===========================================================================
// fnv1a edge cases
// ===========================================================================

TEST_CASE("fnv1a: empty string returns FNV offset basis")
{
    CHECK(fnv1a("") == 0x811c9dc5u);
}

TEST_CASE("fnv1a: single character")
{
    // 'a' should give a deterministic non-zero result different from offset basis
    auto h = fnv1a("a");
    CHECK(h != 0x811c9dc5u);
    CHECK(h == fnv1a("a")); // deterministic
}

TEST_CASE("fnv1a: long string")
{
    // Build a 1000-char string
    std::string long_str(1000, 'x');
    auto h = fnv1a(long_str.c_str());
    CHECK(h == fnv1a(long_str.c_str())); // deterministic
    CHECK(h != fnv1a("")); // different from empty
}

TEST_CASE("fnv1a: common ROS topics do not collide")
{
    const char* topics[] = {
        "/imu", "/gps", "/camera/image_raw", "/odom", "/cmd_vel",
        "/scan", "/tf", "/tf_static", "/joint_states", "/diagnostics",
        "/clock", "/map", "/amcl_pose", "/goal_pose", "/path",
        "/pointcloud", "/depth/image_raw", "/rgb/image_raw",
    };
    constexpr std::size_t n = sizeof(topics) / sizeof(topics[0]);

    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            INFO("Collision between \"" << topics[i] << "\" and \"" << topics[j] << "\"");
            CHECK(fnv1a(topics[i]) != fnv1a(topics[j]));
        }
    }
}

// ===========================================================================
// Ros2MsgData edge cases
// ===========================================================================

TEST_CASE("Ros2MsgData: max serialized_len = MAX_SERIALIZED_SIZE full round-trip")
{
    GstBuffer* buf = gst_buffer_new_allocate(nullptr, 64, nullptr);

    Ros2MsgData data{};
    data.recv_stamp_ns = 1;
    data.msg_stamp_ns  = 2;
    data.topic_hash    = fnv1a("/big");
    data.serialized_len = MAX_SERIALIZED_SIZE;

    // Fill entire 4096 bytes with a pattern
    for (std::size_t i = 0; i < MAX_SERIALIZED_SIZE; ++i) {
        data.serialized[i] = static_cast<std::uint8_t>(i & 0xFF);
    }

    auto* s = Ros2MsgMeta::add(buf, data);
    REQUIRE(s != nullptr);

    auto got = Ros2MsgMeta::get(buf);
    REQUIRE(got.has_value());
    CHECK(got->serialized_len == MAX_SERIALIZED_SIZE);

    bool all_match = true;
    for (std::size_t i = 0; i < MAX_SERIALIZED_SIZE; ++i) {
        if (got->serialized[i] != static_cast<std::uint8_t>(i & 0xFF)) {
            all_match = false;
            break;
        }
    }
    CHECK(all_match);

    gst_buffer_unref(buf);
}

TEST_CASE("Ros2MsgData: zero-length serialized data")
{
    GstBuffer* buf = gst_buffer_new_allocate(nullptr, 64, nullptr);

    Ros2MsgData data{};
    data.recv_stamp_ns = 10;
    data.msg_stamp_ns  = 20;
    data.topic_hash    = fnv1a("/empty");
    data.serialized_len = 0;

    auto* s = Ros2MsgMeta::add(buf, data);
    REQUIRE(s != nullptr);

    auto got = Ros2MsgMeta::get(buf);
    REQUIRE(got.has_value());
    CHECK(got->serialized_len == 0);
    CHECK(got->recv_stamp_ns == 10);
    CHECK(got->msg_stamp_ns == 20);

    gst_buffer_unref(buf);
}

TEST_CASE("Ros2MsgData: extreme field values")
{
    GstBuffer* buf = gst_buffer_new_allocate(nullptr, 64, nullptr);

    Ros2MsgData data{};
    data.recv_stamp_ns  = UINT64_MAX;
    data.msg_stamp_ns   = UINT64_MAX;
    data.topic_hash     = UINT32_MAX;
    data.serialized_len = 1;
    data.serialized[0]  = 0xFF;

    auto* s = Ros2MsgMeta::add(buf, data);
    REQUIRE(s != nullptr);

    auto got = Ros2MsgMeta::get(buf);
    REQUIRE(got.has_value());
    CHECK(got->recv_stamp_ns == UINT64_MAX);
    CHECK(got->msg_stamp_ns == UINT64_MAX);
    CHECK(got->topic_hash == UINT32_MAX);
    CHECK(got->serialized_len == 1);
    CHECK(got->serialized[0] == 0xFF);

    gst_buffer_unref(buf);
}

// ===========================================================================
// Multiple metadata on same buffer
// ===========================================================================

TEST_CASE("100 metadata on same buffer: count and get_all ordering")
{
    GstBuffer* buf = gst_buffer_new_allocate(nullptr, 64, nullptr);

    for (int i = 0; i < 100; ++i) {
        Ros2MsgData data{};
        data.recv_stamp_ns  = static_cast<std::uint64_t>(i);
        data.msg_stamp_ns   = static_cast<std::uint64_t>(i * 10);
        data.topic_hash     = static_cast<std::uint32_t>(i);
        data.serialized_len = 1;
        data.serialized[0]  = static_cast<std::uint8_t>(i & 0xFF);
        Ros2MsgMeta::add(buf, data);
    }

    CHECK(Ros2MsgMeta::count(buf) == 100);

    auto all = Ros2MsgMeta::get_all(buf);
    REQUIRE(all.size() == 100);

    // Verify ordering matches insertion order
    for (int i = 0; i < 100; ++i) {
        CHECK(all[static_cast<std::size_t>(i)].recv_stamp_ns ==
              static_cast<std::uint64_t>(i));
        CHECK(all[static_cast<std::size_t>(i)].topic_hash ==
              static_cast<std::uint32_t>(i));
    }

    gst_buffer_unref(buf);
}

// ===========================================================================
// Metadata survives gst_buffer_copy
// ===========================================================================

TEST_CASE("metadata survives gst_buffer_copy")
{
    GstBuffer* buf = gst_buffer_new_allocate(nullptr, 64, nullptr);

    Ros2MsgData data{};
    data.recv_stamp_ns  = 999;
    data.msg_stamp_ns   = 888;
    data.topic_hash     = fnv1a("/copy_test");
    data.serialized_len = 3;
    data.serialized[0]  = 0xAA;
    data.serialized[1]  = 0xBB;
    data.serialized[2]  = 0xCC;

    Ros2MsgMeta::add(buf, data);

    GstBuffer* copy = gst_buffer_copy(buf);
    REQUIRE(copy != nullptr);

    auto got = Ros2MsgMeta::get(copy);
    REQUIRE(got.has_value());
    CHECK(got->recv_stamp_ns == 999);
    CHECK(got->msg_stamp_ns == 888);
    CHECK(got->topic_hash == fnv1a("/copy_test"));
    CHECK(got->serialized_len == 3);
    CHECK(got->serialized[0] == 0xAA);
    CHECK(got->serialized[1] == 0xBB);
    CHECK(got->serialized[2] == 0xCC);

    gst_buffer_unref(copy);
    gst_buffer_unref(buf);
}

// ===========================================================================
// Metadata copy isolation
// ===========================================================================

TEST_CASE("metadata copy isolation: modify copy, original unchanged")
{
    GstBuffer* buf = gst_buffer_new_allocate(nullptr, 64, nullptr);

    Ros2MsgData data{};
    data.recv_stamp_ns  = 100;
    data.topic_hash     = fnv1a("/original");
    data.serialized_len = 1;
    data.serialized[0]  = 0x11;

    Ros2MsgMeta::add(buf, data);

    GstBuffer* copy = gst_buffer_copy(buf);
    REQUIRE(copy != nullptr);

    // Modify the copy's metadata via get_mut
    auto* mut = Ros2MsgMeta::get_mut(copy);
    REQUIRE(mut != nullptr);
    mut->data.recv_stamp_ns  = 999;
    mut->data.topic_hash     = fnv1a("/modified");
    mut->data.serialized[0]  = 0xFF;

    // Original should be unchanged
    auto orig = Ros2MsgMeta::get(buf);
    REQUIRE(orig.has_value());
    CHECK(orig->recv_stamp_ns == 100);
    CHECK(orig->topic_hash == fnv1a("/original"));
    CHECK(orig->serialized[0] == 0x11);

    // Copy should have the new values
    auto copied = Ros2MsgMeta::get(copy);
    REQUIRE(copied.has_value());
    CHECK(copied->recv_stamp_ns == 999);
    CHECK(copied->topic_hash == fnv1a("/modified"));
    CHECK(copied->serialized[0] == 0xFF);

    gst_buffer_unref(copy);
    gst_buffer_unref(buf);
}

// ===========================================================================
// Null buffer handling
// ===========================================================================

TEST_CASE("null buffer handling for all Ros2MsgMeta API methods")
{
    // add returns nullptr on null buffer
    Ros2MsgData data{};
    CHECK(Ros2MsgMeta::add(nullptr, data) == nullptr);

    // get returns nullopt on null buffer
    CHECK_FALSE(Ros2MsgMeta::get(nullptr).has_value());

    // get_mut returns nullptr on null buffer
    CHECK(Ros2MsgMeta::get_mut(nullptr) == nullptr);

    // count returns 0 on null buffer (for_each guards with g_return_if_fail)
    CHECK(Ros2MsgMeta::count(nullptr) == 0);

    // remove returns false on null buffer
    CHECK_FALSE(Ros2MsgMeta::remove(nullptr));

    // get_all returns empty on null buffer
    auto all = Ros2MsgMeta::get_all(nullptr);
    CHECK(all.empty());
}

// ===========================================================================
// Empty (zero-byte) buffer can hold metadata
// ===========================================================================

TEST_CASE("empty buffer (0 bytes) can hold Ros2MsgMeta")
{
    GstBuffer* buf = gst_buffer_new();
    REQUIRE(buf != nullptr);
    CHECK(gst_buffer_get_size(buf) == 0);

    Ros2MsgData data{};
    data.recv_stamp_ns = 42;
    data.serialized_len = 0;

    auto* s = Ros2MsgMeta::add(buf, data);
    REQUIRE(s != nullptr);

    auto got = Ros2MsgMeta::get(buf);
    REQUIRE(got.has_value());
    CHECK(got->recv_stamp_ns == 42);

    gst_buffer_unref(buf);
}

// ===========================================================================
// SyncBuffer edge cases
// ===========================================================================

TEST_CASE("SyncBuffer: pick from empty returns nullopt for Latest")
{
    SyncBuffer buf(SyncBuffer::Policy::Latest);
    auto r = buf.pick(0, 0);
    CHECK_FALSE(r.has_value());
}

TEST_CASE("SyncBuffer: pick from empty returns nullopt for Nearest")
{
    SyncBuffer buf(SyncBuffer::Policy::Nearest);
    auto r = buf.pick(500, 1000);
    CHECK_FALSE(r.has_value());
}

TEST_CASE("SyncBuffer: capacity=1, rapid push overwrites")
{
    SyncBuffer buf(SyncBuffer::Policy::Latest, 0, /*capacity=*/1);

    for (int i = 0; i < 100; ++i) {
        StampedBlob b;
        b.recv_stamp_ns = static_cast<std::uint64_t>(i);
        b.msg_stamp_ns  = static_cast<std::uint64_t>(i);
        b.cdr           = {static_cast<std::uint8_t>(i & 0xFF)};
        buf.push(std::move(b));
    }

    CHECK(buf.size() == 1);

    auto r = buf.pick(0, 200);
    REQUIRE(r.has_value());
    CHECK(r->recv_stamp_ns == 99);
    CHECK(r->cdr[0] == 99);
}

TEST_CASE("SyncBuffer: nearest with all same timestamp")
{
    SyncBuffer buf(SyncBuffer::Policy::Nearest);

    for (int i = 0; i < 10; ++i) {
        StampedBlob b;
        b.recv_stamp_ns = static_cast<std::uint64_t>(i);
        b.msg_stamp_ns  = 5000; // all same msg_stamp
        b.cdr           = {static_cast<std::uint8_t>(i)};
        buf.push(std::move(b));
    }

    // All have distance 0 to query 5000; should return the first one found
    auto r = buf.pick(5000, 10000);
    REQUIRE(r.has_value());
    // The find_nearest returns the first with minimal distance
    CHECK(r->msg_stamp_ns == 5000);
}

TEST_CASE("SyncBuffer: max_age=0 means no pruning")
{
    SyncBuffer buf(SyncBuffer::Policy::Latest, /*max_age_ns=*/0);

    StampedBlob b;
    b.recv_stamp_ns = 1; // very old
    b.msg_stamp_ns  = 1;
    b.cdr           = {0x42};
    buf.push(std::move(b));

    // Even with a huge "now" timestamp, entry is not pruned
    auto r = buf.pick(0, UINT64_MAX);
    REQUIRE(r.has_value());
    CHECK(r->cdr[0] == 0x42);
    CHECK(buf.size() == 1);
}

// ===========================================================================
// Add/remove/re-add cycle
// ===========================================================================

TEST_CASE("add/remove/re-add cycle for Ros2MsgMeta")
{
    GstBuffer* buf = gst_buffer_new_allocate(nullptr, 64, nullptr);

    // Add
    Ros2MsgData data{};
    data.recv_stamp_ns = 1;
    data.topic_hash = fnv1a("/cycle");
    Ros2MsgMeta::add(buf, data);
    CHECK(Ros2MsgMeta::count(buf) == 1);

    // Remove
    CHECK(Ros2MsgMeta::remove(buf));
    CHECK(Ros2MsgMeta::count(buf) == 0);
    CHECK_FALSE(Ros2MsgMeta::get(buf).has_value());

    // Re-add with different data
    Ros2MsgData data2{};
    data2.recv_stamp_ns = 2;
    data2.topic_hash = fnv1a("/cycle2");
    data2.serialized_len = 1;
    data2.serialized[0] = 0xBB;
    Ros2MsgMeta::add(buf, data2);

    CHECK(Ros2MsgMeta::count(buf) == 1);
    auto got = Ros2MsgMeta::get(buf);
    REQUIRE(got.has_value());
    CHECK(got->recv_stamp_ns == 2);
    CHECK(got->topic_hash == fnv1a("/cycle2"));
    CHECK(got->serialized[0] == 0xBB);

    gst_buffer_unref(buf);
}

// ===========================================================================
// Filter by topic_hash
// ===========================================================================

TEST_CASE("filter by topic_hash: /imu and /gps counting")
{
    GstBuffer* buf = gst_buffer_new_allocate(nullptr, 64, nullptr);

    const auto imu_hash = fnv1a("/imu");
    const auto gps_hash = fnv1a("/gps");

    // Add 5 /imu and 3 /gps metadata
    for (int i = 0; i < 5; ++i) {
        Ros2MsgData data{};
        data.topic_hash = imu_hash;
        data.recv_stamp_ns = static_cast<std::uint64_t>(i);
        Ros2MsgMeta::add(buf, data);
    }
    for (int i = 0; i < 3; ++i) {
        Ros2MsgData data{};
        data.topic_hash = gps_hash;
        data.recv_stamp_ns = static_cast<std::uint64_t>(100 + i);
        Ros2MsgMeta::add(buf, data);
    }

    CHECK(Ros2MsgMeta::count(buf) == 8);

    // Count per topic by iterating
    std::size_t imu_count = 0;
    std::size_t gps_count = 0;
    Ros2MsgMeta::for_each(buf, [&](const Ros2MsgData& d) {
        if (d.topic_hash == imu_hash) ++imu_count;
        if (d.topic_hash == gps_hash) ++gps_count;
    });

    CHECK(imu_count == 5);
    CHECK(gps_count == 3);

    gst_buffer_unref(buf);
}
