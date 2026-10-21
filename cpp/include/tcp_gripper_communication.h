#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

namespace robot::platform::tcp_data::gripper_communication {
// v1: version, configured slot count, flags, reserved. No native struct layout.
struct State {
    std::uint8_t configured_count = 0;
    bool enabled = true;
    // Last known driver connection, not proof of live feedback while paused.
    bool driver_connected = false;
    bool awaiting_command = false;
    bool feedback_fresh = false;
};
inline std::vector<std::uint8_t> encode(const State& state) {
    return {1, state.configured_count,
        static_cast<std::uint8_t>((state.enabled ? 1 : 0) |
            (state.driver_connected ? 2 : 0) | (state.awaiting_command ? 4 : 0) | (state.feedback_fresh ? 8 : 0)), 0};
}
inline bool decode(const std::uint8_t* bytes, std::size_t size, State& out) {
    if (!bytes || size != 4 || bytes[0] != 1 || bytes[1] > 2 || (bytes[2] & ~15) || bytes[3]) return false;
    out = {bytes[1], (bytes[2] & 1) != 0, (bytes[2] & 2) != 0, (bytes[2] & 4) != 0, (bytes[2] & 8) != 0};
    return true;
}
}
