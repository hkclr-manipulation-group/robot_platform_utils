#ifndef CORE_UDP_DISCOVERY_H
#define CORE_UDP_DISCOVERY_H

#include <cstdint>
#include <string>
#include <vector>

namespace robot::platform {

/** Result of probing the network with SdkHandshakeReq. */
struct DiscoveredServer {
    std::string server_ip;
    uint32_t session_id = 0;
};

/**
 * Find the RT core server by sending SdkHandshakeReq and reading the reply peer address.
 *
 * Tries each probe_ip in order. Prefer multicast (same group as telemetry) over
 * 255.255.255.255 — Linux does not loop limited-broadcast back to local sockets, so
 * same-host panel+RT discovery fails with global broadcast.
 *
 * @param client_id         Panel / SDK client id (HMAC + session bookkeeping).
 * @param core_request_port Destination UDP port (same as CoreUdpServer local_port).
 * @param local_ack_port    Local bind port for the handshake reply (same as CoreUdpClient ack port).
 * @param probe_ips         Unicast, multicast, and/or 255.255.255.255 targets.
 * @param timeout_ms        Per-attempt wait for SdkHandshakeRes.
 * @param max_attempts_per_probe Retries per probe IP before moving on.
 *
 * Closes its temporary socket before returning so CoreUdpClient can bind local_ack_port.
 */
DiscoveredServer discoverRtServerViaHandshake(
    uint16_t client_id,
    int core_request_port,
    int local_ack_port,
    const std::vector<std::string>& probe_ips,
    float timeout_ms = 1500.0f,
    int max_attempts_per_probe = 2);

/** Convenience overload for a single probe address. */
inline DiscoveredServer discoverRtServerViaHandshake(
    uint16_t client_id,
    int core_request_port,
    int local_ack_port,
    const std::string& probe_ip,
    float timeout_ms = 1500.0f,
    int max_attempts_per_probe = 2) {
    return discoverRtServerViaHandshake(
        client_id, core_request_port, local_ack_port,
        std::vector<std::string>{probe_ip}, timeout_ms, max_attempts_per_probe);
}

}  // namespace robot::platform

#endif
