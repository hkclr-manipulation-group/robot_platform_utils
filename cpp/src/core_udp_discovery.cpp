#include "core_udp_discovery.h"

#include <cstring>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#if defined(_WIN32) || defined(_WIN64)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
typedef unsigned long in_addr_t; 
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#endif

#include "client_keys.h"
#include "curi_udp/c/curi_udp.h"
#include "platform_serialization.h"
#include "platform_state.h"
#include "time_utils.h"

namespace robot::platform {
namespace {

bool enableBroadcast(curi_socket_t fd) {
#if defined(_WIN32) || defined(_WIN64)
    BOOL opt = TRUE;
    return setsockopt(fd, SOL_SOCKET, SO_BROADCAST, reinterpret_cast<const char*>(&opt), sizeof(opt)) == 0;
#else
    int opt = 1;
    return setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &opt, sizeof(opt)) == 0;
#endif
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
        throw std::runtime_error("discoverRtServerViaHandshake: inet_ntop failed");
    }
    return std::string(buf);
}

}  // namespace

DiscoveredServer discoverRtServerViaHandshake(
    uint16_t client_id,
    int core_request_port,
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
    if (max_attempts_per_probe < 1) {
        max_attempts_per_probe = 1;
    }

    const std::size_t send_buffer_size =
        sizeof(CoreRequestVariant) + HMAC_KEY_SIZE;
    const std::size_t ack_buffer_size =
        sizeof(CoreResponseVariant) + HMAC_KEY_SIZE;

    // Bind once; only the send destination changes per probe.
    udp_node node{};
    const char local_ip[] = "0.0.0.0";
    const int init_ret = udp_init_share_fd(
        &node, local_ip, local_ack_port, probe_ips.front().c_str(), core_request_port,
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

    // Limited broadcast needs SO_BROADCAST; harmless if also probing unicast/multicast.
    if (!enableBroadcast(node.send_fd)) {
        std::cout << "discoverRtServerViaHandshake: warning: SO_BROADCAST failed\n";
    }

    constexpr uint16_t kHandshakeSeq = 1;
    SdkHandshakeReq handshake{};
    handshake.client_id = client_id;
    handshake.sequence_id = kHandshakeSeq;

    std::ostringstream tried;
    for (size_t pi = 0; pi < probe_ips.size(); ++pi) {
        const std::string& probe_ip = probe_ips[pi];
        if (probe_ip.empty()) {
            continue;
        }
        if (!setSendDestination(node, probe_ip, core_request_port)) {
            std::cout << "discoverRtServerViaHandshake: invalid probe_ip " << probe_ip << "\n";
            continue;
        }
        if (pi > 0) {
            tried << ", ";
        }
        tried << probe_ip;

        // if (isLimitedBroadcast(probe_ip)) {
        //     std::cout << "discoverRtServerViaHandshake: note: 255.255.255.255 often fails "
        //                  "for same-host (no broadcast loopback); prefer multicast probe\n";
        // }

        for (int attempt = 1; attempt <= max_attempts_per_probe; ++attempt) {
            handshake.timestamp_us = static_cast<uint64_t>(get_time_now());
            CoreRequestVariantPtr request =
                std::make_unique<CoreRequestVariant>(handshake);

            std::size_t written_size = 0;
            if (!serialization::packMessage(
                    request, node.send_buffer, send_buffer_size, written_size, getClientHmacKey)) {
                throw std::runtime_error("discoverRtServerViaHandshake: packMessage failed");
            }
            udp_send(&node, static_cast<int>(written_size));

            const int timeout_us = static_cast<int>(timeout_ms * 1000.0f);
            sockaddr_in peer{};
            const int n = udp_select1(&node, timeout_us, static_cast<int>(ack_buffer_size), &peer);
            if (n <= 0) {
                std::cout << "discoverRtServerViaHandshake: probe " << probe_ip
                          << " attempt " << attempt << "/" << max_attempts_per_probe
                          << " timed out\n";
                continue;
            }

            CoreResponseVariantPtr ack;
            if (!serialization::unpackMessage(
                    node.receive_buffer, static_cast<std::size_t>(n), ack, getClientHmacKey)
                || !ack
                || !std::holds_alternative<SdkHandshakeRes>(*ack)) {
                std::cout << "discoverRtServerViaHandshake: probe " << probe_ip
                          << " attempt " << attempt << " got unexpected / invalid response\n";
                continue;
            }

            const auto& res = std::get<SdkHandshakeRes>(*ack);
            if (res.request_client_id != client_id) {
                continue;
            }
            if (res.payload.status != CommandResponseStatus::kSuccess) {
                throw std::runtime_error(
                    "discoverRtServerViaHandshake: server rejected handshake ("
                    + enumToString(res.payload.status) + ")");
            }

            DiscoveredServer out;
            out.server_ip = sockaddrToIp(peer);
            out.session_id = res.assigned_session_id;
            // Prefer a unicast peer address for later CoreUdpClient traffic.
            if (isMulticastIp(out.server_ip) || isLimitedBroadcast(out.server_ip)) {
                std::cout << "discoverRtServerViaHandshake: warning: peer address "
                          << out.server_ip << " is not unicast\n";
            }
            std::cout << "discoverRtServerViaHandshake: found server " << out.server_ip
                      << " via probe " << probe_ip
                      << " session_id=" << out.session_id << "\n";
            return out;
        }
    }

    throw std::runtime_error(
        "discoverRtServerViaHandshake: no SdkHandshakeRes from probes [" + tried.str() + "]");
}

}  // namespace robot::platform
