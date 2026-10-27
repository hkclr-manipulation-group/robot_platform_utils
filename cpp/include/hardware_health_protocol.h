#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

namespace robot::platform::health_wire {

constexpr std::uint16_t kVersion = 1;
constexpr std::size_t kMetricCount = 9, kMaxJoints = 7, kMaxArms = 2;
constexpr std::size_t kHeaderSize = 48, kJointSize = 152, kSignatureSize = 16;
constexpr std::size_t kMaxPayload = kHeaderSize + kMaxJoints * kJointSize;
constexpr std::size_t kMaxDatagram = kMaxPayload + kSignatureSize;
constexpr std::uint32_t kMagic = 0x544c4843; // CHLT, little endian
constexpr std::uint32_t kFrequencyHz = 50;
static_assert(kMaxDatagram <= 1200);
static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559);

// Metric order: motor_current_a, torque_nm, temperature_c, bus_voltage_v,
// bus_current_a, driver_status, driver_error, brake_released, iq_raw.
struct Sample { double value = std::numeric_limits<double>::quiet_NaN(); std::uint64_t sampled_us = 0; };
struct Joint { std::uint32_t supported = 0, driver = 0; std::array<Sample, kMetricCount> samples{}; };
struct Packet {
    std::uint16_t client_id = 0;
    std::uint8_t arm_index = 0, arm_count = 0, joint_count = 0, flags = 0;
    std::uint32_t session_id = 0;
    std::uint64_t sequence = 0, captured_us = 0, sent_us = 0;
    std::array<Joint, kMaxJoints> joints{};
};

namespace detail {
inline void put(std::uint8_t*& out, std::uint64_t value, unsigned count) {
    for (unsigned i = 0; i < count; ++i) *out++ = static_cast<std::uint8_t>(value >> (8 * i));
}
inline std::uint64_t get(const std::uint8_t*& in, unsigned count) {
    std::uint64_t value = 0;
    for (unsigned i = 0; i < count; ++i) value |= std::uint64_t(*in++) << (8 * i);
    return value;
}
inline bool valid(const Packet& p) {
    return p.client_id && p.session_id && p.arm_count && p.arm_count <= kMaxArms &&
           p.arm_index < p.arm_count && p.joint_count <= kMaxJoints && !(p.flags & ~3u);
}
} // namespace detail

// Encodes only the payload. Transport MUST append the existing 16-byte session
// client-key HMAC; receivers MUST authenticate before interpreting this payload.
inline bool encode(const Packet& p, std::uint8_t* buffer, std::size_t capacity, std::size_t& size) {
    size = 0;
    const auto needed = kHeaderSize + p.joint_count * kJointSize;
    if (!buffer || !detail::valid(p) || capacity < needed) return false;
    auto* out = buffer;
    using detail::put;
    put(out, kMagic, 4); put(out, kVersion, 2); put(out, kHeaderSize, 2);
    put(out, p.client_id, 2); put(out, p.arm_index, 1); put(out, p.arm_count, 1);
    put(out, p.joint_count, 1); put(out, p.flags, 1); put(out, 0, 2);
    put(out, p.session_id, 4); put(out, 0, 4);
    put(out, p.sequence, 8); put(out, p.captured_us, 8); put(out, p.sent_us, 8);
    for (std::size_t j = 0; j < p.joint_count; ++j) {
        put(out, p.joints[j].supported, 4); put(out, p.joints[j].driver, 4);
        for (const auto& sample : p.joints[j].samples) {
            std::uint64_t bits;
            std::memcpy(&bits, &sample.value, 8);
            put(out, bits, 8); put(out, sample.sampled_us, 8);
        }
    }
    size = needed;
    return true;
}

inline bool decode(const std::uint8_t* buffer, std::size_t size, Packet& packet) {
    if (!buffer || size < kHeaderSize || size > kMaxPayload) return false;
    const auto* in = buffer;
    using detail::get;
    if (get(in, 4) != kMagic || get(in, 2) != kVersion || get(in, 2) != kHeaderSize) return false;
    Packet p;
    p.client_id = static_cast<std::uint16_t>(get(in, 2));
    p.arm_index = static_cast<std::uint8_t>(get(in, 1)); p.arm_count = static_cast<std::uint8_t>(get(in, 1));
    p.joint_count = static_cast<std::uint8_t>(get(in, 1)); p.flags = static_cast<std::uint8_t>(get(in, 1));
    if (get(in, 2)) return false;
    p.session_id = static_cast<std::uint32_t>(get(in, 4));
    if (get(in, 4)) return false;
    p.sequence = get(in, 8); p.captured_us = get(in, 8); p.sent_us = get(in, 8);
    if (!detail::valid(p) || size != kHeaderSize + p.joint_count * kJointSize) return false;
    for (std::size_t j = 0; j < p.joint_count; ++j) {
        p.joints[j].supported = static_cast<std::uint32_t>(get(in, 4));
        p.joints[j].driver = static_cast<std::uint32_t>(get(in, 4));
        if ((p.joints[j].supported >> kMetricCount) || p.joints[j].driver > 2) return false;
        for (auto& sample : p.joints[j].samples) {
            const auto bits = get(in, 8);
            std::memcpy(&sample.value, &bits, 8);
            sample.sampled_us = get(in, 8);
        }
    }
    packet = p; // Malformed packets leave the caller's previous sample intact.
    return true;
}

} // namespace robot::platform::health_wire
