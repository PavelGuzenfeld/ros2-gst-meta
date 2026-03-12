#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <ros2_gst_meta/serialized_meta.hpp>
#include <ros2_gst_meta/sync_buffer.hpp>

using namespace ros2gstmeta;

namespace {

StampedBlob make_blob(std::uint64_t recv_ns, std::uint64_t msg_ns,
                      std::uint8_t tag = 0)
{
    StampedBlob b;
    b.recv_stamp_ns = recv_ns;
    b.msg_stamp_ns  = msg_ns;
    b.cdr           = {tag};
    return b;
}

} // namespace

// ---------------------------------------------------------------------------
// fnv1a
// ---------------------------------------------------------------------------

TEST_CASE("fnv1a: deterministic and collision-resistant")
{
    CHECK(fnv1a("/imu") == fnv1a("/imu"));
    CHECK(fnv1a("/imu") != fnv1a("/gps"));
    CHECK(fnv1a("") == 0x811c9dc5u);
}

// ---------------------------------------------------------------------------
// SyncBuffer — Latest policy
// ---------------------------------------------------------------------------

TEST_CASE("latest policy: returns most recent message")
{
    SyncBuffer buf(SyncBuffer::Policy::Latest);

    buf.push(make_blob(100, 100, 1));
    buf.push(make_blob(200, 200, 2));
    buf.push(make_blob(300, 300, 3));

    auto result = buf.pick(0, 400);
    REQUIRE(result.has_value());
    CHECK(result->cdr[0] == 3);
}

TEST_CASE("latest policy: empty buffer returns nullopt")
{
    SyncBuffer buf(SyncBuffer::Policy::Latest);
    auto result = buf.pick(0, 0);
    CHECK_FALSE(result.has_value());
}

TEST_CASE("latest policy: single message")
{
    SyncBuffer buf(SyncBuffer::Policy::Latest);
    buf.push(make_blob(100, 100, 42));

    auto result = buf.pick(0, 200);
    REQUIRE(result.has_value());
    CHECK(result->cdr[0] == 42);
}

// ---------------------------------------------------------------------------
// SyncBuffer — Nearest policy
// ---------------------------------------------------------------------------

TEST_CASE("nearest policy: picks closest timestamp")
{
    SyncBuffer buf(SyncBuffer::Policy::Nearest);

    buf.push(make_blob(100, 1000, 1));  // msg_stamp=1000
    buf.push(make_blob(200, 2000, 2));  // msg_stamp=2000
    buf.push(make_blob(300, 3000, 3));  // msg_stamp=3000

    SUBCASE("query near first")
    {
        auto r = buf.pick(1100, 400);
        REQUIRE(r.has_value());
        CHECK(r->cdr[0] == 1);
    }

    SUBCASE("query near middle")
    {
        auto r = buf.pick(1900, 400);
        REQUIRE(r.has_value());
        CHECK(r->cdr[0] == 2);
    }

    SUBCASE("query near last")
    {
        auto r = buf.pick(2800, 400);
        REQUIRE(r.has_value());
        CHECK(r->cdr[0] == 3);
    }

    SUBCASE("exact match")
    {
        auto r = buf.pick(2000, 400);
        REQUIRE(r.has_value());
        CHECK(r->cdr[0] == 2);
    }
}

// ---------------------------------------------------------------------------
// SyncBuffer — Max-age pruning
// ---------------------------------------------------------------------------

TEST_CASE("max-age pruning removes stale messages")
{
    SyncBuffer buf(SyncBuffer::Policy::Latest, /*max_age_ns=*/1000);

    buf.push(make_blob(100, 100, 1));
    buf.push(make_blob(200, 200, 2));
    buf.push(make_blob(800, 800, 3));

    // At now=1300, messages with recv_stamp < 300 are stale (age > 1000)
    auto result = buf.pick(0, 1300);
    REQUIRE(result.has_value());
    CHECK(result->cdr[0] == 3);
    CHECK(buf.size() == 1);
}

TEST_CASE("max-age prunes everything → nullopt")
{
    SyncBuffer buf(SyncBuffer::Policy::Latest, /*max_age_ns=*/100);

    buf.push(make_blob(10, 10, 1));

    auto result = buf.pick(0, 1000);
    CHECK_FALSE(result.has_value());
    CHECK(buf.size() == 0);
}

// ---------------------------------------------------------------------------
// SyncBuffer — Capacity
// ---------------------------------------------------------------------------

TEST_CASE("capacity limit evicts oldest entries")
{
    SyncBuffer buf(SyncBuffer::Policy::Latest, 0, /*capacity=*/3);

    buf.push(make_blob(1, 1, 1));
    buf.push(make_blob(2, 2, 2));
    buf.push(make_blob(3, 3, 3));
    buf.push(make_blob(4, 4, 4)); // evicts blob #1

    CHECK(buf.size() == 3);
    auto r = buf.pick(0, 5);
    REQUIRE(r.has_value());
    CHECK(r->cdr[0] == 4); // latest
}

// ---------------------------------------------------------------------------
// SyncBuffer — Policy switching
// ---------------------------------------------------------------------------

TEST_CASE("set_policy changes behavior")
{
    SyncBuffer buf(SyncBuffer::Policy::Latest);

    buf.push(make_blob(100, 1000, 1));
    buf.push(make_blob(200, 2000, 2));
    buf.push(make_blob(300, 3000, 3));

    // Latest: always returns last
    auto r1 = buf.pick(1100, 400);
    REQUIRE(r1.has_value());
    CHECK(r1->cdr[0] == 3);

    // Switch to nearest
    buf.set_policy(SyncBuffer::Policy::Nearest);
    auto r2 = buf.pick(1100, 400);
    REQUIRE(r2.has_value());
    CHECK(r2->cdr[0] == 1);
}

// ---------------------------------------------------------------------------
// SyncBuffer — Stress
// ---------------------------------------------------------------------------

TEST_CASE("stress: push 10000 entries with capacity 100")
{
    SyncBuffer buf(SyncBuffer::Policy::Latest, 0, 100);

    for (int i = 0; i < 10000; ++i) {
        buf.push(make_blob(static_cast<uint64_t>(i),
                           static_cast<uint64_t>(i),
                           static_cast<uint8_t>(i & 0xFF)));
    }

    CHECK(buf.size() == 100);
    auto r = buf.pick(0, 20000);
    REQUIRE(r.has_value());
    CHECK(r->recv_stamp_ns == 9999);
}

// ---------------------------------------------------------------------------
// Ros2MsgData / Ros2MsgMeta round-trip (no ROS 2 needed)
// ---------------------------------------------------------------------------

namespace {
struct GstInitGuard {
    GstInitGuard() { gst_init(nullptr, nullptr); }
};
static GstInitGuard gst_guard; // NOLINT
} // namespace

TEST_CASE("Ros2MsgMeta add/get round-trip")
{
    GstBuffer* buf = gst_buffer_new_allocate(nullptr, 64, nullptr);

    Ros2MsgData data{};
    data.recv_stamp_ns = 42;
    data.msg_stamp_ns  = 100;
    data.topic_hash    = fnv1a("/imu");
    data.serialized_len = 5;
    data.serialized[0] = 0xDE;
    data.serialized[1] = 0xAD;
    data.serialized[2] = 0xBE;
    data.serialized[3] = 0xEF;
    data.serialized[4] = 0x42;

    auto* s = Ros2MsgMeta::add(buf, data);
    REQUIRE(s != nullptr);

    auto got = Ros2MsgMeta::get(buf);
    REQUIRE(got.has_value());
    CHECK(got->recv_stamp_ns == 42);
    CHECK(got->msg_stamp_ns == 100);
    CHECK(got->topic_hash == fnv1a("/imu"));
    CHECK(got->serialized_len == 5);
    CHECK(got->serialized[0] == 0xDE);
    CHECK(got->serialized[4] == 0x42);

    gst_buffer_unref(buf);
}

TEST_CASE("multiple Ros2MsgMeta on same buffer (different topics)")
{
    GstBuffer* buf = gst_buffer_new_allocate(nullptr, 64, nullptr);

    Ros2MsgData imu_data{};
    imu_data.topic_hash = fnv1a("/imu");
    imu_data.serialized_len = 1;
    imu_data.serialized[0] = 0xAA;

    Ros2MsgData gps_data{};
    gps_data.topic_hash = fnv1a("/gps");
    gps_data.serialized_len = 1;
    gps_data.serialized[0] = 0xBB;

    Ros2MsgMeta::add(buf, imu_data);
    Ros2MsgMeta::add(buf, gps_data);

    CHECK(Ros2MsgMeta::count(buf) == 2);

    auto all = Ros2MsgMeta::get_all(buf);
    REQUIRE(all.size() == 2);
    CHECK(all[0].topic_hash == fnv1a("/imu"));
    CHECK(all[0].serialized[0] == 0xAA);
    CHECK(all[1].topic_hash == fnv1a("/gps"));
    CHECK(all[1].serialized[0] == 0xBB);

    gst_buffer_unref(buf);
}
