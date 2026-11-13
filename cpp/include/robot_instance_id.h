#pragma once
#include <cstddef>
#include <string>

namespace robot::platform::robot_identity {
// Deployment identity, independent of sessions, configuration and model hashes.
inline bool validUuid(const std::string& id) {
    if (id.size() != 36 || id[14] != '4' ||
        (id[19] != '8' && id[19] != '9' && id[19] != 'a' && id[19] != 'b')) return false;
    for (std::size_t i = 0; i < id.size(); ++i) {
        if (i == 8 || i == 13 || i == 18 || i == 23) { if (id[i] != '-') return false; }
        else if (!((id[i] >= '0' && id[i] <= '9') || (id[i] >= 'a' && id[i] <= 'f'))) return false;
    }
    return true;
}
} // namespace robot::platform::robot_identity
