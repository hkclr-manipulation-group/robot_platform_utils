#include "core_udp_discovery.h"

#include <atomic>
#include <algorithm>
#include <chrono>
#include <cstring>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#if defined(_WIN32) || defined(_WIN64)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#include <mswsock.h>
#include <iphlpapi.h>
typedef unsigned long in_addr_t;
#else
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netinet/in.h>
#include <sys/socket.h>
#endif

#include "client_keys.h"
#include "curi_udp/c/curi_udp.h"
#include "platform_serialization.h"
#include "platform_state.h"
#include "time_sync.h"
#include "time_utils.h"

namespace robot::platform {
namespace {

struct LocalIpv4Network {
    std::string host_ip;
    std::string broadcast_ip;
    std::string network_ip;
    uint32_t prefix_len = 32;
};

struct DiscoveryTarget {
    std::string dest_ip;
    /** When non-empty, set IP_MULTICAST_IF before send (Windows multi-NIC). */
    std::string multicast_if_ip;
};

std::atomic<uint32_t> g_next_discovery_handshake_seq{1};

uint32_t nextDiscoveryHandshakeSequenceId() {
    return g_next_discovery_handshake_seq.fetch_add(1, std::memory_order_relaxed);
}

/**
 * Advance the discovery sequence by `jump` and return the new value.
 *
 * Servers reject handshakes whose sequence is not strictly newer than the
 * last sequence seen for the client (anti-replay), and that state outlives
 * SDK process restarts for as long as the server keeps the session. A fresh
 * process restarts at 1, so after any session that pushed the server's
 * counter past the resync budget a +1 climb can never catch up. The server
 * accepts anything within (last, last + 2^31), so jumping forward
 * exponentially lands inside the acceptance window after a few rejections.
 */
uint32_t jumpDiscoveryHandshakeSequenceId(uint32_t jump) {
    return g_next_discovery_handshake_seq.fetch_add(jump, std::memory_order_relaxed) + jump;
}

bool isStaleDiscoveryHandshakeResponse(
    const SdkHandshakeRes& res,
    uint16_t client_id,
    uint32_t sequence_id) {
    return res.request_client_id != client_id
        || res.request_sequence_id != sequence_id;
}

std::string wireRobotNameToString(const char (&name)[MAX_NAME_SIZE]) {
    return std::string(name, ::strnlen(name, MAX_NAME_SIZE));
}

void fillDiscoveredServerFromHandshake(
    DiscoveredServer& out,
    const SdkHandshakeRes& res,
    uint32_t handshake_sequence_id,
    int64_t time_offset_us,
    int64_t handshake_rtt_us) {
    out.session_id = res.assigned_session_id;
    out.last_handshake_sequence_id = handshake_sequence_id;
    out.time_offset_us = time_offset_us;
    out.handshake_rtt_us = handshake_rtt_us;
    out.robot_name = wireRobotNameToString(res.payload.robot_name);
}

bool enableBroadcast(curi_socket_t fd) {
#if defined(_WIN32) || defined(_WIN64)
    BOOL opt = TRUE;
    return setsockopt(fd, SOL_SOCKET, SO_BROADCAST, reinterpret_cast<const char*>(&opt), sizeof(opt)) == 0;
#else
    int opt = 1;
    return setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &opt, sizeof(opt)) == 0;
#endif
}

void disableUdpConnReset(curi_socket_t fd) {
#if defined(_WIN32) || defined(_WIN64)
    BOOL disable = FALSE;
    DWORD bytes_returned = 0;
    WSAIoctl(
        fd,
        SIO_UDP_CONNRESET,
        &disable,
        sizeof(disable),
        nullptr,
        0,
        &bytes_returned,
        nullptr,
        nullptr);
#endif
}

bool setMulticastTtl(curi_socket_t fd, int ttl) {
    const char opt = static_cast<char>(ttl);
    return setsockopt(fd, IPPROTO_IP, IP_MULTICAST_TTL, &opt, sizeof(opt)) == 0;
}

bool setMulticastLoop(curi_socket_t fd, bool enabled) {
#if defined(_WIN32) || defined(_WIN64)
    BOOL opt = enabled ? TRUE : FALSE;
    return setsockopt(
        fd, IPPROTO_IP, IP_MULTICAST_LOOP,
        reinterpret_cast<const char*>(&opt), sizeof(opt)) == 0;
#else
    unsigned char opt = enabled ? 1 : 0;
    return setsockopt(fd, IPPROTO_IP, IP_MULTICAST_LOOP, &opt, sizeof(opt)) == 0;
#endif
}

bool setMulticastInterface(curi_socket_t fd, const std::string& if_ip) {
    in_addr addr{};
    if (inet_pton(AF_INET, if_ip.c_str(), &addr) != 1) {
        return false;
    }
    return setsockopt(
        fd, IPPROTO_IP, IP_MULTICAST_IF,
        reinterpret_cast<const char*>(&addr), sizeof(addr)) == 0;
}

void configureDiscoverySocket(curi_socket_t fd) {
    if (!enableBroadcast(fd)) {
        std::cout << "core_udp_discovery: warning: SO_BROADCAST failed\n";
    }
    if (!setMulticastTtl(fd, 1)) {
        std::cout << "core_udp_discovery: warning: IP_MULTICAST_TTL failed\n";
    }
    if (!setMulticastLoop(fd, true)) {
        std::cout << "core_udp_discovery: warning: IP_MULTICAST_LOOP failed\n";
    }
    disableUdpConnReset(fd);
}

bool isLimitedBroadcast(const std::string& ip) {
    return ip == "255.255.255.255";
}

bool isMulticastIp(const std::string& ip) {
    const in_addr_t addr = inet_addr(ip.c_str());
    if (addr == static_cast<in_addr_t>(-1)) {
        return false;
    }
    return IN_MULTICAST(ntohl(addr));
}

bool setSendDestination(udp_node& node, const std::string& ip, int port) {
    memset(&node.send_addr, 0, sizeof(node.send_addr));
    node.send_addr.sin_family = AF_INET;
    node.send_addr.sin_port = htons(static_cast<uint16_t>(port));
    node.CURI_SEND_PORT = port;
    return inet_pton(AF_INET, ip.c_str(), &node.send_addr.sin_addr) == 1;
}

std::string sockaddrToIp(const sockaddr_in& addr) {
    char buf[INET_ADDRSTRLEN] = {};
    if (inet_ntop(AF_INET, &addr.sin_addr, buf, sizeof(buf)) == nullptr) {
        throw std::runtime_error("core_udp_discovery: inet_ntop failed");
    }
    return std::string(buf);
}

std::string ipv4ToString(const in_addr& addr) {
    char buf[INET_ADDRSTRLEN] = {};
    if (inet_ntop(AF_INET, &addr, buf, sizeof(buf)) == nullptr) {
        return {};
    }
    return std::string(buf);
}

std::string broadcastFromHostAndPrefix(const in_addr& host, uint32_t prefix_len) {
    if (prefix_len > 32) {
        return {};
    }
    const uint32_t host_be = ntohl(host.s_addr);
    const uint32_t mask = prefix_len == 0 ? 0u : (~0u << (32 - prefix_len));
    const uint32_t bcast_be = (host_be & mask) | (~mask);
    in_addr bcast{};
    bcast.s_addr = htonl(bcast_be);
    return ipv4ToString(bcast);
}

std::string networkFromHostAndPrefix(const in_addr& host, uint32_t prefix_len) {
    if (prefix_len > 32) {
        return {};
    }
    const uint32_t host_be = ntohl(host.s_addr);
    const uint32_t mask = prefix_len == 0 ? 0u : (~0u << (32 - prefix_len));
    in_addr net{};
    net.s_addr = htonl(host_be & mask);
    return ipv4ToString(net);
}

uint32_t prefixLenFromNetmask(const in_addr& netmask) {
    const uint32_t mask_be = ntohl(netmask.s_addr);
    uint32_t prefix_len = 0;
    for (uint32_t bit = 0; bit < 32; ++bit) {
        if ((mask_be & (1u << (31 - bit))) != 0) {
            prefix_len++;
        } else {
            break;
        }
    }
    return prefix_len;
}

std::vector<LocalIpv4Network> collectLocalIpv4Networks() {
    std::vector<LocalIpv4Network> networks;

#if defined(_WIN32) || defined(_WIN64)
    ULONG buffer_size = 15000;
    std::vector<unsigned char> buffer(buffer_size);
    IP_ADAPTER_ADDRESSES* adapters =
        reinterpret_cast<IP_ADAPTER_ADDRESSES*>(buffer.data());
    const ULONG flags =
        GAA_FLAG_SKIP_ANYCAST | GAA_FLAG_SKIP_MULTICAST | GAA_FLAG_SKIP_DNS_SERVER;

    ULONG result = GetAdaptersAddresses(
        AF_INET, flags, nullptr, adapters, &buffer_size);
    if (result == ERROR_BUFFER_OVERFLOW) {
        buffer.resize(buffer_size);
        adapters = reinterpret_cast<IP_ADAPTER_ADDRESSES*>(buffer.data());
        result = GetAdaptersAddresses(
            AF_INET, flags, nullptr, adapters, &buffer_size);
    }
    if (result != NO_ERROR) {
        return networks;
    }

    for (IP_ADAPTER_ADDRESSES* adapter = adapters;
         adapter != nullptr;
         adapter = adapter->Next) {
        if (adapter->OperStatus != IfOperStatusUp) {
            continue;
        }
        for (IP_ADAPTER_UNICAST_ADDRESS* unicast = adapter->FirstUnicastAddress;
             unicast != nullptr;
             unicast = unicast->Next) {
            if (!unicast->Address.lpSockaddr
                || unicast->Address.lpSockaddr->sa_family != AF_INET) {
                continue;
            }
            const auto* sin =
                reinterpret_cast<const sockaddr_in*>(unicast->Address.lpSockaddr);
            const std::string host_ip = ipv4ToString(sin->sin_addr);
            if (host_ip.empty() || host_ip == "0.0.0.0") {
                continue;
            }
            const uint32_t prefix_len = unicast->OnLinkPrefixLength;
            const std::string broadcast_ip = broadcastFromHostAndPrefix(
                sin->sin_addr, prefix_len);
            if (broadcast_ip.empty()) {
                continue;
            }
            networks.push_back({host_ip, broadcast_ip,
                                networkFromHostAndPrefix(sin->sin_addr, prefix_len),
                                prefix_len});
        }
    }
#else
    ifaddrs* ifap = nullptr;
    if (getifaddrs(&ifap) != 0) {
        return networks;
    }

    for (ifaddrs* ifa = ifap; ifa != nullptr; ifa = ifa->ifa_next) {
        if (!ifa->ifa_addr || ifa->ifa_addr->sa_family != AF_INET) {
            continue;
        }
        if ((ifa->ifa_flags & IFF_UP) == 0 || (ifa->ifa_flags & IFF_LOOPBACK) != 0) {
            continue;
        }
        const auto* sin = reinterpret_cast<const sockaddr_in*>(ifa->ifa_addr);
        const std::string host_ip = ipv4ToString(sin->sin_addr);
        if (host_ip.empty()) {
            continue;
        }

        uint32_t prefix_len = 32;
        if (ifa->ifa_netmask && ifa->ifa_netmask->sa_family == AF_INET) {
            const auto* nmask = reinterpret_cast<const sockaddr_in*>(ifa->ifa_netmask);
            prefix_len = prefixLenFromNetmask(nmask->sin_addr);
        }

        std::string broadcast_ip;
        if (ifa->ifa_broadaddr && ifa->ifa_broadaddr->sa_family == AF_INET) {
            const auto* bsin = reinterpret_cast<const sockaddr_in*>(ifa->ifa_broadaddr);
            broadcast_ip = ipv4ToString(bsin->sin_addr);
        }
        if (broadcast_ip.empty()) {
            broadcast_ip = broadcastFromHostAndPrefix(sin->sin_addr, prefix_len);
        }
        if (broadcast_ip.empty()) {
            continue;
        }
        networks.push_back({host_ip, broadcast_ip,
                            networkFromHostAndPrefix(sin->sin_addr, prefix_len),
                            prefix_len});
    }
    freeifaddrs(ifap);
#endif

    return networks;
}

std::vector<DiscoveryTarget> buildDiscoveryTargets(
    const std::vector<std::string>& probe_ips,
    bool expand_local_broadcasts,
    bool include_subnet_sweep = false) {
    std::vector<DiscoveryTarget> targets;
    std::unordered_set<std::string> seen;

    auto addTarget = [&](const std::string& dest_ip,
                         const std::string& multicast_if_ip = std::string{}) {
        if (dest_ip.empty()) {
            return;
        }
        const std::string key = dest_ip + "|" + multicast_if_ip;
        if (!seen.insert(key).second) {
            return;
        }
        targets.push_back({dest_ip, multicast_if_ip});
    };

    const auto local_networks = collectLocalIpv4Networks();

    for (const std::string& probe_ip : probe_ips) {
        if (probe_ip.empty()) {
            continue;
        }
        if (isMulticastIp(probe_ip)) {
            addTarget(probe_ip);
            for (const LocalIpv4Network& net : local_networks) {
                if (net.host_ip != "127.0.0.1") {
                    addTarget(probe_ip, net.host_ip);
                }
            }
            continue;
        }
        addTarget(probe_ip);
    }
    if (expand_local_broadcasts) {
        for (const LocalIpv4Network& net : local_networks) {
            if (net.broadcast_ip != "255.255.255.255") {
                addTarget(net.broadcast_ip);
            }
        }
    }

    // Unicast sweep of small local subnets. Many WiFi networks (hotspots,
    // APs with client isolation / multicast filtering) never deliver
    // broadcast or multicast frames, while unicast still works — that is why
    // direct-IP connect keeps working when discovery does not. Sweeping the
    // subnet with unicast handshakes finds servers on such networks.
    // Only subnets with at most a /24 worth of hosts are swept so the burst
    // stays bounded.
    if (include_subnet_sweep) {
        for (const LocalIpv4Network& net : local_networks) {
            if (net.host_ip.empty() || net.host_ip == "127.0.0.1") {
                continue;
            }
            uint32_t sweep_prefix = net.prefix_len;
            // WiFi interfaces often report /32 when the OS lacks a netmask;
            // fall back to a typical /24 LAN so sweep still runs.
            if (sweep_prefix > 30 || sweep_prefix < 24) {
                if (sweep_prefix == 32) {
                    sweep_prefix = 24;
                } else {
                    continue;
                }
            }
            in_addr host_addr{};
            if (inet_pton(AF_INET, net.host_ip.c_str(), &host_addr) != 1) {
                continue;
            }
            const uint32_t host_be = ntohl(host_addr.s_addr);
            const uint32_t mask = ~0u << (32 - sweep_prefix);
            const uint32_t first_be = host_be & mask;
            const uint32_t last_be = first_be | ~mask;
            for (uint32_t ip_be = first_be; ip_be < last_be; ++ip_be) {
                if (ip_be == first_be || ip_be == last_be || ip_be == host_be) {
                    continue;  // skip network, broadcast, and our own address
                }
                in_addr addr{};
                addr.s_addr = htonl(ip_be);
                addTarget(ipv4ToString(addr));
            }
        }
    }

    return targets;
}

bool prepareDiscoverySend(
    udp_node& node,
    const DiscoveryTarget& target,
    int core_request_port) {
    if (!setSendDestination(node, target.dest_ip, core_request_port)) {
        return false;
    }
    if (!target.multicast_if_ip.empty()) {
        if (!setMulticastInterface(node.send_fd, target.multicast_if_ip)) {
            std::cout << "core_udp_discovery: IP_MULTICAST_IF failed for "
                      << target.multicast_if_ip << "\n";
            return false;
        }
    }
    return true;
}

bool sendDiscoveryPacket(udp_node& node, int packet_size) {
    const int sent = sendto(
        node.send_fd,
        reinterpret_cast<const char*>(node.send_buffer),
        packet_size,
        0,
        reinterpret_cast<const sockaddr*>(&node.send_addr),
        sizeof(node.send_addr));
#if defined(_WIN32) || defined(_WIN64)
    if (sent == SOCKET_ERROR) {
        std::cout << "core_udp_discovery: sendto " << inet_ntoa(node.send_addr.sin_addr)
                  << ":" << ntohs(node.send_addr.sin_port)
                  << " failed, error=" << WSAGetLastError() << "\n";
        return false;
    }
#else
    if (sent < 0) {
        std::cout << "core_udp_discovery: sendto failed\n";
        return false;
    }
#endif
    return sent == packet_size;
}

void sendDiscoveryHandshakeBurst(
    udp_node& node,
    const std::vector<DiscoveryTarget>& targets,
    int core_request_port,
    SdkHandshakeReq& handshake,
    std::size_t send_buffer_size) {
    handshake.timestamp_us = static_cast<uint64_t>(get_time_now());

    for (const DiscoveryTarget& target : targets) {
        if (!prepareDiscoverySend(node, target, core_request_port)) {
            continue;
        }

        CoreRequestVariantPtr packed =
            std::make_unique<CoreRequestVariant>(handshake);

        std::size_t written_size = 0;
        if (!serialization::packMessage(
                packed, node.send_buffer, send_buffer_size, written_size, getClientHmacKey)) {
            throw std::runtime_error("core_udp_discovery: packMessage failed");
        }
        sendDiscoveryPacket(node, static_cast<int>(written_size));
    }
}

bool isProbeResponseAllowed(
    const std::string& peer_ip,
    const std::vector<std::string>& probe_ips,
    bool expand_local_broadcasts) {
    bool broadcast_style_probe = false;
    for (const std::string& probe_ip : probe_ips) {
        if (peer_ip == probe_ip) {
            return true;
        }
        if (isMulticastIp(probe_ip) || isLimitedBroadcast(probe_ip)) {
            broadcast_style_probe = true;
        }
    }
    // A broadcast/multicast probe may legitimately be answered by any host on
    // the network. A unicast probe (direct-IP connect) must only be answered
    // by its target — otherwise connect("A") can silently land on server B.
    return expand_local_broadcasts && broadcast_style_probe;
}

/** True when the probe list targets the network at large (multicast or
 *  limited broadcast) rather than a specific server. Only then does a
 *  subnet unicast sweep make sense. */
bool probeListIncludesBroadcastStyleTarget(const std::vector<std::string>& probe_ips) {
    for (const std::string& probe_ip : probe_ips) {
        if (isMulticastIp(probe_ip) || isLimitedBroadcast(probe_ip)) {
            return true;
        }
    }
    return false;
}

bool isLoopbackIp(const std::string& ip) {
    return ip == "127.0.0.1";
}

bool isExplicitLoopbackProbe(const std::vector<std::string>& probe_ips) {
    return probe_ips.size() == 1 && probe_ips[0] == "127.0.0.1";
}

bool shouldIgnoreLoopbackDiscoveryReply(
    const std::string& server_ip,
    const std::vector<std::string>& probe_ips) {
    return isLoopbackIp(server_ip) && !isExplicitLoopbackProbe(probe_ips);
}

std::string makeDiscoveryMergeKey(const DiscoveredServer& server) {
    return std::to_string(server.session_id) + '|' + server.robot_name;
}

void mergeDiscoveredServerEntry(
    DiscoveredServer& existing,
    const DiscoveredServer& incoming) {
    existing.last_handshake_sequence_id = std::max(
        existing.last_handshake_sequence_id,
        incoming.last_handshake_sequence_id);
    if (!incoming.robot_name.empty()) {
        existing.robot_name = incoming.robot_name;
    }

    const bool incoming_is_loopback = isLoopbackIp(incoming.server_ip);
    const bool existing_is_loopback = isLoopbackIp(existing.server_ip);
    if (incoming_is_loopback && !existing_is_loopback) {
        return;
    }
    if (!incoming_is_loopback && existing_is_loopback) {
        existing.server_ip = incoming.server_ip;
        existing.time_offset_us = incoming.time_offset_us;
        existing.handshake_rtt_us = incoming.handshake_rtt_us;
        return;
    }

    if (incoming.handshake_rtt_us > 0
        && (existing.handshake_rtt_us <= 0
            || incoming.handshake_rtt_us < existing.handshake_rtt_us)) {
        existing.server_ip = incoming.server_ip;
        existing.time_offset_us = incoming.time_offset_us;
        existing.handshake_rtt_us = incoming.handshake_rtt_us;
    }
}

constexpr float kDiscoveryIdleGraceMs = 300.0f;
constexpr float kBroadcastProbeMaxListenMs = 1500.0f;
constexpr float kSubnetSweepListenMs = 2000.0f;
constexpr std::size_t kSubnetSweepBatchSize = 32;

float listenTimeoutMs(float base_timeout_ms, std::size_t target_count) {
    if (target_count > 32) {
        return kSubnetSweepListenMs;
    }
    return std::min(base_timeout_ms, kBroadcastProbeMaxListenMs);
}

struct DiscoveryListenState {
    bool saw_sequence_rejection = false;
    bool have_any_reply = false;
    bool have_lan_reply = false;
    std::chrono::steady_clock::time_point last_valid_reply_at{};
};

bool shouldStopDiscoveryListen(
    const DiscoveryListenState& state,
    bool subnet_sweep_phase,
    std::chrono::steady_clock::time_point now) {
    if (!state.have_any_reply) {
        return false;
    }
    if (subnet_sweep_phase && !state.have_lan_reply) {
        return false;
    }
    const auto idle_deadline = state.last_valid_reply_at
        + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<float, std::milli>(kDiscoveryIdleGraceMs));
    return now >= idle_deadline;
}

bool shouldExpandToSubnetSweep(
    bool can_sweep,
    bool sweep_expanded,
    int attempt,
    const std::unordered_map<std::string, DiscoveredServer>& by_identity,
    const DiscoveryListenState& prev_listen_state) {
    if (!can_sweep || sweep_expanded || attempt <= 1) {
        return false;
    }
    if (!by_identity.empty()) {
        return false;
    }
    // After the first broadcast/multicast attempt, widen to unicast subnet sweep
    // when no LAN robot was found (matches discoverRtServerViaHandshake timeout path).
    if (!prev_listen_state.have_any_reply) {
        return true;
    }
    return prev_listen_state.have_any_reply && !prev_listen_state.have_lan_reply;
}

void listenForDiscoveryRepliesUntil(
    udp_node& node,
    const SdkHandshakeReq& handshake,
    uint16_t client_id,
    std::size_t ack_buffer_size,
    std::unordered_map<std::string, DiscoveredServer>& by_identity,
    DiscoveryListenState& state,
    bool subnet_sweep_phase,
    std::chrono::steady_clock::time_point deadline) {
    while (true) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) {
            break;
        }
        if (shouldStopDiscoveryListen(state, subnet_sweep_phase, now)) {
            std::cout << "discoverAllRtServersViaHandshake: early exit after "
                      << kDiscoveryIdleGraceMs << " ms idle\n";
            break;
        }

        auto select_until = deadline;
        if (state.have_any_reply && (!subnet_sweep_phase || state.have_lan_reply)) {
            const auto idle_deadline = state.last_valid_reply_at
                + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<float, std::milli>(kDiscoveryIdleGraceMs));
            select_until = std::min(select_until, idle_deadline);
        }
        const auto remaining = select_until - now;
        const auto remaining_us =
            std::chrono::duration_cast<std::chrono::microseconds>(remaining).count();
        if (remaining_us <= 0) {
            continue;
        }

        sockaddr_in peer{};
        const int n = udp_select1(
            &node,
            static_cast<int>(remaining_us),
            static_cast<int>(ack_buffer_size),
            &peer);
        if (n <= 0) {
            continue;
        }

        CoreResponseVariantPtr ack;
        if (!serialization::unpackMessage(
                node.receive_buffer, static_cast<std::size_t>(n), ack, getClientHmacKey)
            || !ack
            || !std::holds_alternative<SdkHandshakeRes>(*ack)) {
            continue;
        }
        const auto& res = std::get<SdkHandshakeRes>(*ack);
        if (isStaleDiscoveryHandshakeResponse(res, client_id, handshake.sequence_id)) {
            continue;
        }
        if (res.payload.status == CommandResponseStatus::kRejectedSequenceId) {
            state.saw_sequence_rejection = true;
            continue;
        }
        if (res.payload.status != CommandResponseStatus::kSuccess) {
            throw std::runtime_error(
                "discoverAllRtServersViaHandshake: server rejected handshake ("
                + enumToString(res.payload.status) + ")");
        }

        const int64_t client_recv_us = get_time_now();
        const TimeSyncResult sync = computeTimeSyncFromHandshake(HandshakeTimestamps{
            static_cast<int64_t>(handshake.timestamp_us),
            static_cast<int64_t>(res.request_received_us),
            static_cast<int64_t>(res.response_sent_us),
            client_recv_us,
        });

        DiscoveredServer found;
        found.server_ip = sockaddrToIp(peer);
        fillDiscoveredServerFromHandshake(
            found,
            res,
            handshake.sequence_id,
            sanitizeTimeSyncOffset(sync.offset_us),
            sync.rtt_us);
        if (isMulticastIp(found.server_ip) || isLimitedBroadcast(found.server_ip)) {
            continue;
        }
        state.last_valid_reply_at = std::chrono::steady_clock::now();
        state.have_any_reply = true;
        if (isLoopbackIp(found.server_ip)) {
            std::cout << "discoverAllRtServersViaHandshake: ignoring loopback reply from "
                      << "local rt_control (robot_name=" << found.robot_name << ")\n";
            continue;
        }
        state.have_lan_reply = true;
        std::cout << "discoverAllRtServersViaHandshake: reply from "
                  << found.server_ip << " robot_name=" << found.robot_name
                  << " rtt_us=" << found.handshake_rtt_us << "\n";
        const std::string merge_key = makeDiscoveryMergeKey(found);
        auto existing = by_identity.find(merge_key);
        if (existing == by_identity.end()) {
            by_identity.emplace(merge_key, found);
        } else {
            mergeDiscoveredServerEntry(existing->second, found);
        }
    }
}

void probeDiscoveryTargets(
    udp_node& node,
    const std::vector<DiscoveryTarget>& targets,
    int core_request_port,
    SdkHandshakeReq& handshake,
    std::size_t send_buffer_size,
    std::size_t ack_buffer_size,
    uint16_t client_id,
    std::unordered_map<std::string, DiscoveredServer>& by_identity,
    DiscoveryListenState& state,
    float listen_timeout_ms) {
    const bool subnet_sweep_phase = targets.size() > 32;
    const auto deadline =
        std::chrono::steady_clock::now()
        + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<float, std::milli>(listen_timeout_ms));

    if (!subnet_sweep_phase) {
        sendDiscoveryHandshakeBurst(
            node, targets, core_request_port, handshake, send_buffer_size);
        listenForDiscoveryRepliesUntil(
            node, handshake, client_id, ack_buffer_size, by_identity, state,
            subnet_sweep_phase, deadline);
        return;
    }

    // Subnet sweep: send in batches and listen between batches so replies are
    // not stuck behind hundreds of sequential sendto() calls.
    const std::size_t batch_count =
        (targets.size() + kSubnetSweepBatchSize - 1) / kSubnetSweepBatchSize;
    const float batch_listen_ms = listen_timeout_ms / static_cast<float>(batch_count);

    for (std::size_t offset = 0; offset < targets.size(); offset += kSubnetSweepBatchSize) {
        const std::size_t batch_end =
            std::min(offset + kSubnetSweepBatchSize, targets.size());
        const std::vector<DiscoveryTarget> batch(
            targets.begin() + static_cast<std::ptrdiff_t>(offset),
            targets.begin() + static_cast<std::ptrdiff_t>(batch_end));
        sendDiscoveryHandshakeBurst(
            node, batch, core_request_port, handshake, send_buffer_size);

        const auto batch_deadline =
            std::chrono::steady_clock::now()
            + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<float, std::milli>(batch_listen_ms));
        listenForDiscoveryRepliesUntil(
            node, handshake, client_id, ack_buffer_size, by_identity, state,
            subnet_sweep_phase, std::min(batch_deadline, deadline));

        if (shouldStopDiscoveryListen(
                state, subnet_sweep_phase, std::chrono::steady_clock::now())) {
            break;
        }
        if (std::chrono::steady_clock::now() >= deadline) {
            break;
        }
    }
}

}  // namespace

DiscoveredServer discoverRtServerViaHandshake(
    uint16_t client_id,
    int core_request_port,
    int telemetry_port,
    int local_ack_port,
    const std::vector<std::string>& probe_ips,
    float timeout_ms,
    int max_attempts_per_probe,
    bool expand_local_broadcasts) {
    if (probe_ips.empty()) {
        throw std::invalid_argument("discoverRtServerViaHandshake: probe_ips is empty");
    }
    if (core_request_port <= 0 || local_ack_port <= 0) {
        throw std::invalid_argument("discoverRtServerViaHandshake: invalid ports");
    }
    if (telemetry_port <= 0) {
        throw std::invalid_argument("discoverRtServerViaHandshake: telemetry_port must be > 0");
    }
    if (max_attempts_per_probe < 1) {
        max_attempts_per_probe = 1;
    }

    const bool can_sweep = probeListIncludesBroadcastStyleTarget(probe_ips);
    std::vector<DiscoveryTarget> targets =
        buildDiscoveryTargets(probe_ips, expand_local_broadcasts);
    if (targets.empty()) {
        throw std::invalid_argument("discoverRtServerViaHandshake: no discovery targets");
    }
    bool sweep_expanded = false;

    const std::size_t send_buffer_size =
        sizeof(CoreRequestVariant) + HMAC_KEY_SIZE;
    const std::size_t ack_buffer_size =
        sizeof(CoreResponseVariant) + HMAC_KEY_SIZE;

    udp_node node{};
    const char local_ip[] = "0.0.0.0";
    const int init_ret = udp_init_share_fd(
        &node, local_ip, local_ack_port, targets.front().dest_ip.c_str(), core_request_port,
        static_cast<int>(ack_buffer_size), static_cast<int>(send_buffer_size));
    if (init_ret != 0) {
        throw std::runtime_error(
            "discoverRtServerViaHandshake: udp_init_share_fd failed, code "
            + std::to_string(init_ret));
    }

    struct UdpCloser {
        udp_node* n;
        ~UdpCloser() { if (n) udp_close(n); }
    } closer{&node};

    configureDiscoverySocket(node.send_fd);

    SdkHandshakeReq handshake{};
    handshake.client_id = client_id;
    uint32_t sequence_id = nextDiscoveryHandshakeSequenceId();
    int sequence_resync_remaining = 16;
    uint32_t sequence_jump = 1024;
    handshake.telemetry_port = telemetry_port;

    std::ostringstream tried;
    for (size_t ti = 0; ti < targets.size(); ++ti) {
        if (ti > 0) {
            tried << ", ";
        }
        tried << targets[ti].dest_ip;
        if (!targets[ti].multicast_if_ip.empty()) {
            tried << "@" << targets[ti].multicast_if_ip;
        }
    }

    for (int attempt = 1; attempt <= max_attempts_per_probe; ++attempt) {
        handshake.sequence_id = sequence_id;
        sendDiscoveryHandshakeBurst(
            node, targets, core_request_port, handshake, send_buffer_size);

        const int timeout_us = static_cast<int>(timeout_ms * 1000.0f);
        sockaddr_in peer{};
        const int n = udp_select1(&node, timeout_us, static_cast<int>(ack_buffer_size), &peer);
        if (n <= 0) {
            std::cout << "discoverRtServerViaHandshake: attempt " << attempt
                      << "/" << max_attempts_per_probe << " timed out\n";
            if (!sweep_expanded && can_sweep) {
                // Broadcast/multicast went unanswered (common on WiFi APs that
                // filter them) — widen with a unicast sweep of local subnets.
                targets = buildDiscoveryTargets(
                    probe_ips, expand_local_broadcasts, /*include_subnet_sweep=*/true);
                sweep_expanded = true;
                std::cout << "discoverRtServerViaHandshake: expanding to unicast subnet sweep ("
                          << targets.size() << " targets)\n";
            }
            continue;
        }

        CoreResponseVariantPtr ack;
        if (!serialization::unpackMessage(
                node.receive_buffer, static_cast<std::size_t>(n), ack, getClientHmacKey)
            || !ack
            || !std::holds_alternative<SdkHandshakeRes>(*ack)) {
            std::cout << "discoverRtServerViaHandshake: attempt " << attempt
                      << " got unexpected / invalid response\n";
            continue;
        }

        const auto& res = std::get<SdkHandshakeRes>(*ack);
        if (isStaleDiscoveryHandshakeResponse(res, client_id, handshake.sequence_id)) {
            continue;
        }
        if (res.payload.status == CommandResponseStatus::kRejectedSequenceId) {
            if (sequence_resync_remaining-- <= 0) {
                throw std::runtime_error(
                    "discoverRtServerViaHandshake: server rejected handshake ("
                    + enumToString(res.payload.status) + ")");
            }
            sequence_id = jumpDiscoveryHandshakeSequenceId(sequence_jump);
            sequence_jump *= 2;
            if (sequence_jump > (1u << 20)) {
                sequence_jump = 1u << 20;
            }
            --attempt;
            continue;
        }
        if (res.payload.status != CommandResponseStatus::kSuccess) {
            throw std::runtime_error(
                "discoverRtServerViaHandshake: server rejected handshake ("
                + enumToString(res.payload.status) + ")");
        }

        const int64_t client_recv_us = get_time_now();
        const TimeSyncResult sync = computeTimeSyncFromHandshake(HandshakeTimestamps{
            static_cast<int64_t>(handshake.timestamp_us),
            static_cast<int64_t>(res.request_received_us),
            static_cast<int64_t>(res.response_sent_us),
            client_recv_us,
        });

        DiscoveredServer out;
        out.server_ip = sockaddrToIp(peer);
        if (!isProbeResponseAllowed(out.server_ip, probe_ips, expand_local_broadcasts)) {
            std::cout << "discoverRtServerViaHandshake: ignoring response from "
                      << out.server_ip << " (not in probe list)\n";
            continue;
        }
        fillDiscoveredServerFromHandshake(
            out, res, handshake.sequence_id, sanitizeTimeSyncOffset(sync.offset_us), sync.rtt_us);
        if (shouldIgnoreLoopbackDiscoveryReply(out.server_ip, probe_ips)) {
            continue;
        }
        if (isMulticastIp(out.server_ip) || isLimitedBroadcast(out.server_ip)) {
            std::cout << "discoverRtServerViaHandshake: warning: peer address "
                      << out.server_ip << " is not unicast\n";
        }
        std::cout << "discoverRtServerViaHandshake: found server " << out.server_ip
                  << " robot_name=" << out.robot_name
                  << " time_offset_us=" << out.time_offset_us
                  << " rtt_us=" << out.handshake_rtt_us << "\n";
        return out;
    }

    throw std::runtime_error(
        "discoverRtServerViaHandshake: no SdkHandshakeRes from probes [" + tried.str() + "]");
}

std::vector<DiscoveredServer> discoverAllRtServersViaHandshake(
    uint16_t client_id,
    int core_request_port,
    int telemetry_port,
    int local_ack_port,
    const std::vector<std::string>& probe_ips,
    float timeout_ms,
    int max_attempts_per_probe) {
    if (probe_ips.empty()) {
        throw std::invalid_argument("discoverAllRtServersViaHandshake: probe_ips is empty");
    }
    if (core_request_port <= 0 || local_ack_port <= 0) {
        throw std::invalid_argument("discoverAllRtServersViaHandshake: invalid ports");
    }
    if (telemetry_port <= 0) {
        throw std::invalid_argument("discoverAllRtServersViaHandshake: telemetry_port must be > 0");
    }
    if (max_attempts_per_probe < 1) {
        max_attempts_per_probe = 1;
    }

    const bool can_sweep = probeListIncludesBroadcastStyleTarget(probe_ips);
    std::vector<DiscoveryTarget> targets = buildDiscoveryTargets(probe_ips, true);
    if (targets.empty()) {
        return {};
    }
    bool sweep_expanded = false;

    const std::size_t send_buffer_size =
        sizeof(CoreRequestVariant) + HMAC_KEY_SIZE;
    const std::size_t ack_buffer_size =
        sizeof(CoreResponseVariant) + HMAC_KEY_SIZE;

    udp_node node{};
    const char local_ip[] = "0.0.0.0";
    const int init_ret = udp_init_share_fd(
        &node, local_ip, local_ack_port, targets.front().dest_ip.c_str(), core_request_port,
        static_cast<int>(ack_buffer_size), static_cast<int>(send_buffer_size));
    if (init_ret != 0) {
        throw std::runtime_error(
            "discoverAllRtServersViaHandshake: udp_init_share_fd failed, code "
            + std::to_string(init_ret));
    }

    struct UdpCloser {
        udp_node* n;
        ~UdpCloser() { if (n) udp_close(n); }
    } closer{&node};

    configureDiscoverySocket(node.send_fd);

    std::unordered_map<std::string, DiscoveredServer> by_identity;
    uint32_t sequence_id = nextDiscoveryHandshakeSequenceId();
    int sequence_resync_remaining = 16;
    uint32_t sequence_jump = 1024;
    DiscoveryListenState prev_listen_state;

    for (int attempt = 1; attempt <= max_attempts_per_probe; ++attempt) {
        if (shouldExpandToSubnetSweep(
                can_sweep, sweep_expanded, attempt, by_identity, prev_listen_state)) {
            targets = buildDiscoveryTargets(
                probe_ips, /*expand_local_broadcasts=*/true,
                /*include_subnet_sweep=*/true);
            sweep_expanded = true;
            std::cout << "discoverAllRtServersViaHandshake: expanding to unicast subnet sweep ("
                      << targets.size() << " targets)\n";
        }

        SdkHandshakeReq handshake{};
        handshake.client_id = client_id;
        handshake.sequence_id = sequence_id;
        handshake.telemetry_port = telemetry_port;

        const float listen_timeout_ms = listenTimeoutMs(timeout_ms, targets.size());
        std::cout << "discoverAllRtServersViaHandshake: attempt " << attempt
                  << "/" << max_attempts_per_probe << " probing "
                  << targets.size() << " targets, listen "
                  << listen_timeout_ms << " ms\n";

        DiscoveryListenState listen_state;
        probeDiscoveryTargets(
            node,
            targets,
            core_request_port,
            handshake,
            send_buffer_size,
            ack_buffer_size,
            client_id,
            by_identity,
            listen_state,
            listen_timeout_ms);

        if (listen_state.have_any_reply) {
            // logged per reply above
        } else {
            std::cout << "discoverAllRtServersViaHandshake: attempt " << attempt
                      << " got no reply\n";
        }

        if (listen_state.have_lan_reply || !by_identity.empty()) {
            break;
        }
        if (listen_state.have_any_reply && !listen_state.have_lan_reply && can_sweep
            && !sweep_expanded) {
            std::cout << "discoverAllRtServersViaHandshake: only loopback replied; "
                      << "continuing LAN subnet sweep\n";
        }
        prev_listen_state = listen_state;
        if (listen_state.saw_sequence_rejection && sequence_resync_remaining-- > 0) {
            sequence_id = jumpDiscoveryHandshakeSequenceId(sequence_jump);
            sequence_jump *= 2;
            if (sequence_jump > (1u << 20)) {
                sequence_jump = 1u << 20;
            }
            --attempt;
        } else {
            sequence_id = nextDiscoveryHandshakeSequenceId();
        }
    }

    std::vector<DiscoveredServer> out;
    out.reserve(by_identity.size());
    for (auto& entry : by_identity) {
        out.push_back(std::move(entry.second));
    }
    return out;
}

}  // namespace robot::platform
