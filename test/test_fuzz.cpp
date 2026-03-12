#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <ros2_gst_meta/serialized_meta.hpp>
#include <ros2_gst_meta/sync_buffer.hpp>

#include <algorithm>
#include <climits>
#include <cstring>
#include <random>
#include <vector>

using namespace ros2gstmeta;

namespace {
struct GstInitGuard {
    GstInitGuard() { gst_init(nullptr, nullptr); }
};
static GstInitGuard gst_guard; // NOLINT

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

// ===========================================================================
// SyncBuffer: random push/pick sequences
// ===========================================================================

TEST_CASE("fuzz: random push/pick on SyncBuffer (10000 iterations)")
{
    std::mt19937 rng(42); // fixed seed for reproducibility
    std::uniform_int_distribution<int> action_dist(0, 1); // 0=push, 1=pick
    std::uniform_int_distribution<std::uint64_t> ts_dist(0, 1'000'000'000);

    SyncBuffer buf(SyncBuffer::Policy::Latest, 0, 256);

    for (int i = 0; i < 10000; ++i) {
        if (action_dist(rng) == 0) {
            auto ts = ts_dist(rng);
            buf.push(make_blob(ts, ts, static_cast<std::uint8_t>(i & 0xFF)));
        } else {
            auto query = ts_dist(rng);
            auto now   = ts_dist(rng);
            // Should not crash or throw; result may or may not have value
            [[maybe_unused]] auto r = buf.pick(query, now);
        }
    }

    // If we got here without crashing, the test passes
    CHECK(buf.size() <= 256);
}

TEST_CASE("fuzz: random push/pick on SyncBuffer Nearest (10000 iterations)")
{
    std::mt19937 rng(123);
    std::uniform_int_distribution<int> action_dist(0, 1);
    std::uniform_int_distribution<std::uint64_t> ts_dist(0, 1'000'000'000);

    SyncBuffer buf(SyncBuffer::Policy::Nearest, 0, 128);

    for (int i = 0; i < 10000; ++i) {
        if (action_dist(rng) == 0) {
            auto ts = ts_dist(rng);
            buf.push(make_blob(ts, ts, static_cast<std::uint8_t>(i & 0xFF)));
        } else {
            auto query = ts_dist(rng);
            auto now   = ts_dist(rng);
            [[maybe_unused]] auto r = buf.pick(query, now);
        }
    }

    CHECK(buf.size() <= 128);
}

// ===========================================================================
// Ros2MsgMeta: random add/get/remove sequences
// ===========================================================================

TEST_CASE("fuzz: random add/get/remove on Ros2MsgMeta (10000 iterations)")
{
    std::mt19937 rng(77);
    std::uniform_int_distribution<int> action_dist(0, 2); // 0=add, 1=get, 2=remove

    GstBuffer* buf = gst_buffer_new_allocate(nullptr, 64, nullptr);
    std::size_t expected_count = 0;

    for (int i = 0; i < 10000; ++i) {
        int action = action_dist(rng);
        switch (action) {
        case 0: { // add
            Ros2MsgData data{};
            data.recv_stamp_ns = static_cast<std::uint64_t>(i);
            data.topic_hash = static_cast<std::uint32_t>(i);
            data.serialized_len = 1;
            data.serialized[0] = static_cast<std::uint8_t>(i & 0xFF);
            auto* s = Ros2MsgMeta::add(buf, data);
            REQUIRE(s != nullptr);
            ++expected_count;
            break;
        }
        case 1: { // get
            auto got = Ros2MsgMeta::get(buf);
            if (expected_count > 0) {
                CHECK(got.has_value());
            } else {
                CHECK_FALSE(got.has_value());
            }
            break;
        }
        case 2: { // remove
            bool removed = Ros2MsgMeta::remove(buf);
            if (expected_count > 0) {
                CHECK(removed);
                --expected_count;
            } else {
                CHECK_FALSE(removed);
            }
            break;
        }
        }
    }

    CHECK(Ros2MsgMeta::count(buf) == expected_count);

    gst_buffer_unref(buf);
}

// ===========================================================================
// Random serialized data payloads
// ===========================================================================

TEST_CASE("fuzz: random serialized payloads (random lengths 0-4096)")
{
    std::mt19937 rng(999);
    std::uniform_int_distribution<std::uint32_t> len_dist(0, MAX_SERIALIZED_SIZE);
    std::uniform_int_distribution<int> byte_dist(0, 255);

    for (int iter = 0; iter < 200; ++iter) {
        GstBuffer* buf = gst_buffer_new_allocate(nullptr, 16, nullptr);

        Ros2MsgData data{};
        data.serialized_len = len_dist(rng);
        data.recv_stamp_ns  = static_cast<std::uint64_t>(iter);
        data.topic_hash     = static_cast<std::uint32_t>(iter);

        for (std::uint32_t j = 0; j < data.serialized_len; ++j) {
            data.serialized[j] = static_cast<std::uint8_t>(byte_dist(rng));
        }

        auto* s = Ros2MsgMeta::add(buf, data);
        REQUIRE(s != nullptr);

        auto got = Ros2MsgMeta::get(buf);
        REQUIRE(got.has_value());
        CHECK(got->serialized_len == data.serialized_len);

        // Verify all bytes round-trip
        bool match = (std::memcmp(got->serialized, data.serialized,
                                  data.serialized_len) == 0);
        CHECK(match);

        gst_buffer_unref(buf);
    }
}

// ===========================================================================
// Rapid buffer create/populate/copy/destroy
// ===========================================================================

TEST_CASE("fuzz: rapid buffer create/populate/copy/destroy (500 cycles)")
{
    for (int i = 0; i < 500; ++i) {
        GstBuffer* buf = gst_buffer_new_allocate(nullptr, 32, nullptr);

        Ros2MsgData data{};
        data.recv_stamp_ns  = static_cast<std::uint64_t>(i);
        data.msg_stamp_ns   = static_cast<std::uint64_t>(i * 2);
        data.topic_hash     = static_cast<std::uint32_t>(i);
        data.serialized_len = 4;
        data.serialized[0]  = static_cast<std::uint8_t>(i & 0xFF);
        data.serialized[1]  = static_cast<std::uint8_t>((i >> 8) & 0xFF);
        data.serialized[2]  = 0xDE;
        data.serialized[3]  = 0xAD;

        Ros2MsgMeta::add(buf, data);

        GstBuffer* copy = gst_buffer_copy(buf);
        REQUIRE(copy != nullptr);

        auto got = Ros2MsgMeta::get(copy);
        REQUIRE(got.has_value());
        CHECK(got->recv_stamp_ns == static_cast<std::uint64_t>(i));

        gst_buffer_unref(copy);
        gst_buffer_unref(buf);
    }
}

// ===========================================================================
// Burst: add 100, remove all, re-add 100
// ===========================================================================

TEST_CASE("fuzz: burst add 100, remove all, re-add 100 with different data")
{
    GstBuffer* buf = gst_buffer_new_allocate(nullptr, 64, nullptr);

    // Phase 1: add 100
    for (int i = 0; i < 100; ++i) {
        Ros2MsgData data{};
        data.recv_stamp_ns = static_cast<std::uint64_t>(i);
        data.serialized_len = 1;
        data.serialized[0] = 0xAA;
        Ros2MsgMeta::add(buf, data);
    }
    CHECK(Ros2MsgMeta::count(buf) == 100);

    // Phase 2: remove all
    for (int i = 0; i < 100; ++i) {
        CHECK(Ros2MsgMeta::remove(buf));
    }
    CHECK(Ros2MsgMeta::count(buf) == 0);
    CHECK_FALSE(Ros2MsgMeta::remove(buf)); // nothing left

    // Phase 3: re-add 100 with different data
    for (int i = 0; i < 100; ++i) {
        Ros2MsgData data{};
        data.recv_stamp_ns = static_cast<std::uint64_t>(1000 + i);
        data.serialized_len = 1;
        data.serialized[0] = 0xBB;
        Ros2MsgMeta::add(buf, data);
    }
    CHECK(Ros2MsgMeta::count(buf) == 100);

    // Verify all re-added entries have new data
    auto all = Ros2MsgMeta::get_all(buf);
    REQUIRE(all.size() == 100);
    for (std::size_t i = 0; i < 100; ++i) {
        CHECK(all[i].recv_stamp_ns >= 1000);
        CHECK(all[i].serialized[0] == 0xBB);
    }

    gst_buffer_unref(buf);
}

// ===========================================================================
// SyncBuffer stress: push 10000 with capacity 50
// ===========================================================================

TEST_CASE("fuzz: SyncBuffer push 10000 entries capacity 50, size never exceeds 50")
{
    SyncBuffer buf(SyncBuffer::Policy::Latest, 0, /*capacity=*/50);

    for (int i = 0; i < 10000; ++i) {
        buf.push(make_blob(static_cast<std::uint64_t>(i),
                           static_cast<std::uint64_t>(i),
                           static_cast<std::uint8_t>(i & 0xFF)));
        CHECK(buf.size() <= 50);
    }

    CHECK(buf.size() == 50);

    auto r = buf.pick(0, 20000);
    REQUIRE(r.has_value());
    CHECK(r->recv_stamp_ns == 9999); // latest
}

// ===========================================================================
// Concurrent-style: copy isolation (200 iterations)
// ===========================================================================

TEST_CASE("fuzz: copy isolation stress (200 iterations)")
{
    for (int i = 0; i < 200; ++i) {
        GstBuffer* buf = gst_buffer_new_allocate(nullptr, 16, nullptr);

        Ros2MsgData data{};
        data.recv_stamp_ns  = static_cast<std::uint64_t>(i);
        data.topic_hash     = static_cast<std::uint32_t>(i);
        data.serialized_len = 2;
        data.serialized[0]  = static_cast<std::uint8_t>(i & 0xFF);
        data.serialized[1]  = 0x55;

        Ros2MsgMeta::add(buf, data);

        GstBuffer* copy = gst_buffer_copy(buf);
        REQUIRE(copy != nullptr);

        // Modify copy
        auto* mut = Ros2MsgMeta::get_mut(copy);
        REQUIRE(mut != nullptr);
        mut->data.recv_stamp_ns = 0xDEAD;
        mut->data.serialized[0] = 0xFF;

        // Original unchanged
        auto orig = Ros2MsgMeta::get(buf);
        REQUIRE(orig.has_value());
        CHECK(orig->recv_stamp_ns == static_cast<std::uint64_t>(i));
        CHECK(orig->serialized[0] == static_cast<std::uint8_t>(i & 0xFF));

        gst_buffer_unref(copy);
        gst_buffer_unref(buf);
    }
}

// ===========================================================================
// Random topic hashes: add 50 different, filter by each
// ===========================================================================

TEST_CASE("fuzz: 50 random topic hashes, filter by each")
{
    std::mt19937 rng(314);
    std::uniform_int_distribution<std::uint32_t> hash_dist(1, UINT32_MAX);

    GstBuffer* buf = gst_buffer_new_allocate(nullptr, 64, nullptr);

    // Generate 50 unique hashes
    std::vector<std::uint32_t> hashes;
    while (hashes.size() < 50) {
        auto h = hash_dist(rng);
        if (std::find(hashes.begin(), hashes.end(), h) == hashes.end()) {
            hashes.push_back(h);
        }
    }

    // Add 2 metadata per hash = 100 total
    for (auto h : hashes) {
        for (int j = 0; j < 2; ++j) {
            Ros2MsgData data{};
            data.topic_hash     = h;
            data.recv_stamp_ns  = static_cast<std::uint64_t>(h) + static_cast<std::uint64_t>(j);
            data.serialized_len = 0;
            Ros2MsgMeta::add(buf, data);
        }
    }

    CHECK(Ros2MsgMeta::count(buf) == 100);

    // Filter by each hash and verify count
    for (auto h : hashes) {
        std::size_t count = 0;
        Ros2MsgMeta::for_each(buf, [&](const Ros2MsgData& d) {
            if (d.topic_hash == h) ++count;
        });
        CHECK(count == 2);
    }

    gst_buffer_unref(buf);
}

// ===========================================================================
// Edge payloads: special byte patterns
// ===========================================================================

TEST_CASE("fuzz: edge payloads - all zeros")
{
    GstBuffer* buf = gst_buffer_new_allocate(nullptr, 16, nullptr);

    Ros2MsgData data{};
    data.serialized_len = MAX_SERIALIZED_SIZE;
    std::memset(data.serialized, 0x00, MAX_SERIALIZED_SIZE);

    auto* s = Ros2MsgMeta::add(buf, data);
    REQUIRE(s != nullptr);

    auto got = Ros2MsgMeta::get(buf);
    REQUIRE(got.has_value());
    CHECK(got->serialized_len == MAX_SERIALIZED_SIZE);

    bool all_zero = true;
    for (std::size_t i = 0; i < MAX_SERIALIZED_SIZE; ++i) {
        if (got->serialized[i] != 0x00) { all_zero = false; break; }
    }
    CHECK(all_zero);

    gst_buffer_unref(buf);
}

TEST_CASE("fuzz: edge payloads - all 0xFF")
{
    GstBuffer* buf = gst_buffer_new_allocate(nullptr, 16, nullptr);

    Ros2MsgData data{};
    data.serialized_len = MAX_SERIALIZED_SIZE;
    std::memset(data.serialized, 0xFF, MAX_SERIALIZED_SIZE);

    auto* s = Ros2MsgMeta::add(buf, data);
    REQUIRE(s != nullptr);

    auto got = Ros2MsgMeta::get(buf);
    REQUIRE(got.has_value());

    bool all_ff = true;
    for (std::size_t i = 0; i < MAX_SERIALIZED_SIZE; ++i) {
        if (got->serialized[i] != 0xFF) { all_ff = false; break; }
    }
    CHECK(all_ff);

    gst_buffer_unref(buf);
}

TEST_CASE("fuzz: edge payloads - alternating pattern")
{
    GstBuffer* buf = gst_buffer_new_allocate(nullptr, 16, nullptr);

    Ros2MsgData data{};
    data.serialized_len = MAX_SERIALIZED_SIZE;
    for (std::size_t i = 0; i < MAX_SERIALIZED_SIZE; ++i) {
        data.serialized[i] = (i % 2 == 0) ? 0xAA : 0x55;
    }

    auto* s = Ros2MsgMeta::add(buf, data);
    REQUIRE(s != nullptr);

    auto got = Ros2MsgMeta::get(buf);
    REQUIRE(got.has_value());

    bool pattern_ok = true;
    for (std::size_t i = 0; i < MAX_SERIALIZED_SIZE; ++i) {
        std::uint8_t expected = (i % 2 == 0) ? 0xAA : 0x55;
        if (got->serialized[i] != expected) { pattern_ok = false; break; }
    }
    CHECK(pattern_ok);

    gst_buffer_unref(buf);
}
