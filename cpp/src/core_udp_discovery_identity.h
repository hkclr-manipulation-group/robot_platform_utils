#pragma once

#include "core_udp_discovery.h"

namespace robot::platform::detail {

inline std::string discoveryMergeKey(const DiscoveredServer& server) {
    // A discovery result is metadata, not a session. Different robots commonly
    // have the same factory name and all discovery-only replies carry token 0.
    return server.server_ip;
}

}  // namespace robot::platform::detail
