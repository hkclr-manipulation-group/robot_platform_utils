#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include "client_keys.h"
#include "core_udp_client.h"
#include "platform_serialization.h"
#include "platform_state.h"
#include "time_utils.h"

namespace {

using namespace robot::platform;

constexpr uint16_t kClientId = 10002;
const std::string kServerIp = "127.0.0.1";
constexpr int kCoreRequestPort = 30200;
constexpr int kTelemetryPort = 30202;
constexpr int kReceiveAckPort = 40002;
constexpr int kReceiveTimeoutUs = 200000;
constexpr float kAckPollMs = 10.0f;

std::atomic<bool> stopRequested(false);

void handleSig(int)
{
    stopRequested = true;
}

bool unpackSrvState(const std::uint8_t* buffer, std::size_t size, SrvState& out)
{
    out = SrvState{};
    out.magic_header = 0;
    return serialization::unpackMessage(buffer, size, out, getClientHmacKey);
}

bool packCoreRequest(
    const CoreRequestVariantPtr& request,
    std::uint8_t* buffer,
    std::size_t buffer_size,
    std::size_t& written_size)
{
    return serialization::packMessage(request, buffer, buffer_size, written_size, getClientHmacKey);
}

bool unpackCoreResponse(const std::uint8_t* buffer, std::size_t size, CoreResponseVariantPtr& out)
{
    out = nullptr;
    return serialization::unpackMessage(buffer, size, out, getClientHmacKey);
}

std::string resolveKeysPath()
{
#ifdef PROJECT_ROOT_DIR
    return std::string(PROJECT_ROOT_DIR) + "/cuarm_rt_control/src/keys.json";
#else
    return "src/keys.json";
#endif
}

uint32_t performHandshake(CoreUdpClient<SrvState, CoreRequestVariantPtr, CoreResponseVariantPtr>& client)
{
    constexpr uint16_t handshakeSeq = 1;
    constexpr int kMaxAttempts = 3;

    SdkHandshakeReq handshake{};
    handshake.client_id = kClientId;
    handshake.sequence_id = handshakeSeq;
    handshake.telemetry_port = kTelemetryPort;

    for (int attempt = 1; attempt <= kMaxAttempts; ++attempt) {
        handshake.timestamp_us = static_cast<uint64_t>(get_time_now());
        CoreRequestVariantPtr request = std::make_unique<CoreRequestVariant>(handshake);
        client.send(request);

        CoreResponseVariantPtr ack = client.waitAck(kClientId, handshakeSeq, 3000.0f);
        if (!ack || !std::holds_alternative<SdkHandshakeRes>(*ack)) {
            if (attempt == kMaxAttempts) {
                throw std::runtime_error("Handshake failed for client 10002");
            }
            continue;
        }

        const auto& handshakeRes = std::get<SdkHandshakeRes>(*ack);
        if (handshakeRes.payload.status != CommandResponseStatus::kSuccess) {
            throw std::runtime_error(
                "Handshake rejected, status: " + enumToString(handshakeRes.payload.status));
        }

        std::cout << "Client 10002 handshake success on attempt " << attempt
                  << ", session_id=" << handshakeRes.assigned_session_id << std::endl;
        return handshakeRes.assigned_session_id;
    }

    throw std::runtime_error("Handshake failed for client 10002");
}

void sendDemoConfig(
    CoreUdpClient<SrvState, CoreRequestVariantPtr, CoreResponseVariantPtr>& client,
    uint32_t sessionId,
    uint32_t sequenceId,
    bool readOnly)
{
    SdkConfigReq config{};
    config.client_id = kClientId;
    config.session_id = sessionId;
    config.sequence_id = sequenceId;
    config.timestamp_us = static_cast<uint64_t>(get_time_now());
    config.payload.read_only = readOnly;
    CoreRequestVariantPtr request = std::make_unique<CoreRequestVariant>(config);
    client.send(request);

    CoreResponseVariantPtr ack = client.waitAck(kClientId, sequenceId, 3000.0f);
    if (!ack || !std::holds_alternative<SdkConfigRes>(*ack)) {
        throw std::runtime_error("Config ack timed out for client 10002");
    }

    const auto& configRes = std::get<SdkConfigRes>(*ack);
    std::cout << "Client 10002 config response status="
              << enumToString(configRes.payload.status) << std::endl;
    if (configRes.payload.status != CommandResponseStatus::kSuccess) {
        throw std::runtime_error("Config rejected for client 10002");
    }
}

void sendDemoCommand(
    CoreUdpClient<SrvState, CoreRequestVariantPtr, CoreResponseVariantPtr>& client,
    uint32_t sessionId,
    uint32_t sequenceId)
{
    SdkCommandReq command{};
    command.client_id = kClientId;
    command.session_id = sessionId;
    command.sequence_id = sequenceId;
    command.timestamp_us = static_cast<uint64_t>(get_time_now());
    std::snprintf(command.payload.robot_name, sizeof(command.payload.robot_name), "test_robot");
    command.payload.arm_size = 1;
    command.payload.gripper_size = 1;
    command.payload.arm_joint_size[0] = 7;
    command.payload.gripper_joint_size[0] = 1;
    command.payload.enable_jog = 0;
    command.payload.activated_control_strategy = ControlStrategy::kJoint;
    command.payload.target_count = 1;

    CoreRequestVariantPtr request = std::make_unique<CoreRequestVariant>(command);
    client.send(request);

    CoreResponseVariantPtr ack = client.waitAck(kClientId, sequenceId, 3000.0f);
    if (!ack || !std::holds_alternative<SdkCommandRes>(*ack)) {
        throw std::runtime_error("Command ack timed out for client 10002");
    }

    const auto& commandRes = std::get<SdkCommandRes>(*ack);
    std::cout << "Client 10002 command response status="
              << enumToString(commandRes.payload.status) << std::endl;
    if (commandRes.payload.status != CommandResponseStatus::kSuccess) {
        throw std::runtime_error("Command rejected for client 10002");
    }
}

}  // namespace

int main()
{
    std::signal(SIGINT, handleSig);
    uint32_t sequenceId = 1;

    try {
        if (!loadClientKeys(resolveKeysPath())) {
            throw std::runtime_error("Failed to load client keys from " + resolveKeysPath());
        }

        CoreUdpClient<SrvState, CoreRequestVariantPtr, CoreResponseVariantPtr> client(
            kServerIp,
            kCoreRequestPort,
            kTelemetryPort,
            kReceiveAckPort,
            unpackSrvState,
            packCoreRequest,
            unpackCoreResponse,
            kAckPollMs);

        std::cout << "Client 10002 connected to server " << kServerIp << ":" << kCoreRequestPort
                  << ", telemetry port " << kTelemetryPort << std::endl;

        std::cout << "Perform handshake." << std::endl;
        const uint32_t sessionId = performHandshake(client);
        std::cout << "--------------------------------" << std::endl;
        std::cout << "Send demo config, read_only=true." << std::endl;
        sendDemoConfig(client, sessionId, sequenceId++, true);
        std::cout << "--------------------------------" << std::endl;
        std::cout << "Send demo config, read_only=false." << std::endl;
        sendDemoConfig(client, sessionId, sequenceId++, false);
        std::cout << "--------------------------------" << std::endl;
        std::cout << "Send demo command." << std::endl;
        sendDemoCommand(client, sessionId, sequenceId++);
        std::cout << "--------------------------------" << std::endl;

        while (!stopRequested) {
            sendDemoCommand(client, sessionId, sequenceId++);

            SrvState state{};
            if (client.receive(state, kReceiveTimeoutUs)) {
                std::cout << "Client 10002 received SrvState seq=" << state.sequence_id
                          << " system_state=" << enumToString(state.payload.system_state) << std::endl;
            }
        }

        client.close();
        std::cout << "Client 10002 closed." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Client 10002 error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
