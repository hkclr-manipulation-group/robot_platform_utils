#include "core_udp_discovery.h"

#include <atomic>
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
            const std::string broadcast_ip = broadcastFromHostAndPrefix(
                sin->sin_addr, unicast->OnLinkPrefixLength);
            if (broadcast_ip.empty()) {
                continue;
            }
            networks.push_back({host_ip, broadcast_ip});
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

        std::string broadcast_ip;
        if (ifa->ifa_broadaddr && ifa->ifa_broadaddr->sa_family == AF_INET) {
            const auto* bsin = reinterpret_cast<const sockaddr_in*>(ifa->ifa_broadaddr);
            broadcast_ip = ipv4ToString(bsin->sin_addr);
        }
        if (broadcast_ip.empty() && ifa->ifa_netmask
            && ifa->ifa_netmask->sa_family == AF_INET) {
            const auto* nmask = reinterpret_cast<const sockaddr_in*>(ifa->ifa_netmask);
            uint32_t prefix_len = 0;
            const uint32_t mask_be = ntohl(nmask->sin_addr.s_addr);
            for (uint32_t bit = 0; bit < 32; ++bit) {
                if ((mask_be & (1u << (31 - bit))) != 0) {
                    prefix_len++;
                } else {
                    break;
                }
            }
            broadcast_ip = broadcastFromHostAndPrefix(sin->sin_addr, prefix_len);
        }
        if (broadcast_ip.empty()) {
            continue;
        }
        networks.push_back({host_ip, broadcast_ip});
    }
    freeifaddrs(ifap);
#endif

    return networks;
}

std::vector<DiscoveryTarget> buildDiscoveryTargets(
    const std::vector<std::string>& probe_ips) {
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

    for (const LocalIpv4Network& net : local_networks) {
        if (net.broadcast_ip != "255.255.255.255") {
            addTarget(net.broadcast_ip);
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

}  // namespace

DiscoveredServer discoverRtServerViaHandshake(
    uint16_t client_id,
    int core_request_port,
    int telemetry_port,
    int local_ack_port,
    const std::vector<std::string>& probe_ips,
    float timeout_ms,
    int max_attempts_per_probe) {
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

    const std::vector<DiscoveryTarget> targets = buildDiscoveryTargets(probe_ips);
    if (targets.empty()) {
        throw std::invalid_argument("discoverRtServerViaHandshake: no discovery targets");
    }

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
            sequence_id = nextDiscoveryHandshakeSequenceId();
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
        fillDiscoveredServerFromHandshake(
            out, res, handshake.sequence_id, sanitizeTimeSyncOffset(sync.offset_us), sync.rtt_us);
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

    const std::vector<DiscoveryTarget> targets = buildDiscoveryTargets(probe_ips);
    if (targets.empty()) {
        return {};
    }

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

    std::unordered_map<std::string, DiscoveredServer> by_ip;
    uint32_t sequence_id = nextDiscoveryHandshakeSequenceId();
    int sequence_resync_remaining = 16;

    for (int attempt = 1; attempt <= max_attempts_per_probe; ++attempt) {
        SdkHandshakeReq handshake{};
        handshake.client_id = client_id;
        handshake.sequence_id = sequence_id;
        handshake.telemetry_port = telemetry_port;
        sendDiscoveryHandshakeBurst(
            node, targets, core_request_port, handshake, send_buffer_size);

        bool saw_sequence_rejection = false;
        const auto deadline =
            std::chrono::steady_clock::now()
            + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<float, std::milli>(timeout_ms));

        while (std::chrono::steady_clock::now() < deadline) {
            const auto remaining = deadline - std::chrono::steady_clock::now();
            const auto remaining_us =
                std::chrono::duration_cast<std::chrono::microseconds>(remaining).count();
            if (remaining_us <= 0) {
                break;
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
                saw_sequence_rejection = true;
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
            auto existing = by_ip.find(found.server_ip);
            if (existing == by_ip.end()) {
                by_ip.emplace(found.server_ip, found);
            } else {
                existing->second.session_id = found.session_id;
                existing->second.last_handshake_sequence_id = std::max(
                    existing->second.last_handshake_sequence_id,
                    found.last_handshake_sequence_id);
                existing->second.time_offset_us = found.time_offset_us;
                existing->second.handshake_rtt_us = found.handshake_rtt_us;
                if (!found.robot_name.empty()) {
                    existing->second.robot_name = found.robot_name;
                }
            }
        }

        if (!by_ip.empty()) {
            break;
        }
        if (saw_sequence_rejection && sequence_resync_remaining-- > 0) {
            sequence_id = nextDiscoveryHandshakeSequenceId();
            --attempt;
        } else {
            sequence_id = nextDiscoveryHandshakeSequenceId();
        }
    }

    std::vector<DiscoveredServer> out;
    out.reserve(by_ip.size());
    for (auto& entry : by_ip) {
        out.push_back(std::move(entry.second));
    }
    return out;
}

}  // namespace robot::platform
