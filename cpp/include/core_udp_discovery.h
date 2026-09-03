#ifndef CORE_UDP_DISCOVERY_H
#define CORE_UDP_DISCOVERY_H

#include <cstdint>
#include <string>
#include <vector>

namespace robot::platform {

/** Result of probing the network with SdkHandshakeReq. */
struct DiscoveredServer {
    std::string robot_name;
    std::string server_ip;
    uint32_t session_id = 0;
    uint32_t last_handshake_sequence_id = 0;
    /** robot_timestamp_us = local_timestamp_us + time_offset_us (from handshake). */
    int64_t time_offset_us = 0;
    int64_t handshake_rtt_us = 0;
};

/**
 * Find the RT core server by sending SdkHandshakeReq and reading the reply peer address.
 *
 * Tries each probe_ip in order. Prefer multicast (same group as telemetry) over
 * 255.255.255.255 — Linux does not loop limited-broadcast back to local sockets, so
 * same-host panel+RT discovery fails with global broadcast.
 *
 * When the probe list targets the network at large (multicast / 255.255.255.255)
 * and an attempt gets no reply, later attempts are widened with a unicast sweep
 * of small local subnets: many WiFi networks drop broadcast/multicast frames
 * while unicast still gets through (direct-IP connect keeps working there).
 *
 * Handshake sequence rejections from the server's anti-replay check (state that
 * outlives client restarts) are recovered by jumping the sequence forward
 * exponentially instead of retrying with +1.
 *
 * @param client_id         Panel / SDK client id (HMAC + session bookkeeping).
 * @param core_request_port Destination UDP port (same as CoreUdpServer local_port).
 * @param telemetry_port    Telemetry UDP port (same as CoreUdpServer telemetry port).
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
    int telemetry_port,
    int local_ack_port,
    const std::vector<std::string>& probe_ips,
    float timeout_ms = 1500.0f,
    int max_attempts_per_probe = 2,
    bool expand_local_broadcasts = true);

/** Convenience overload for a single probe address. */
inline DiscoveredServer discoverRtServerViaHandshake(
    uint16_t client_id,
    int core_request_port,
    int telemetry_port,
    int local_ack_port,
    const std::string& probe_ip,
    float timeout_ms = 1500.0f,
    int max_attempts_per_probe = 2,
    bool expand_local_broadcasts = true) {
    return discoverRtServerViaHandshake(
        client_id, core_request_port, telemetry_port, local_ack_port,
        std::vector<std::string>{probe_ip}, timeout_ms, max_attempts_per_probe,
        expand_local_broadcasts);
}

/**
 * Enumerate all RT servers that reply to SdkHandshakeReq.
 * Collects unique peer IPs across probe targets within each wait window.
 * When broadcast/multicast probes go unanswered, later attempts add a unicast
 * sweep of local /24-/30 subnets (WiFi APs often block broadcast but allow
 * direct unicast). Returns an empty vector when none respond (does not throw).
 */
std::vector<DiscoveredServer> discoverAllRtServersViaHandshake(
    uint16_t client_id,
    int core_request_port,
    int telemetry_port,
    int local_ack_port,
    const std::vector<std::string>& probe_ips,
    float timeout_ms = 200.0f,
    int max_attempts_per_probe = 2);

}  // namespace robot::platform

#endif
