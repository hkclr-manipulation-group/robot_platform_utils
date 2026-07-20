#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include "client_keys.h"
#include "mcast_client.h"
#include "platform_serialization.h"
#include "time_utils.h"

namespace {

using namespace robot::platform;

constexpr uint16_t kClientId = 10001;
const std::string kServerIp = "127.0.0.1";
const std::string kMulticastIp = "239.255.77.88";
constexpr int kServerPort = 30200;
constexpr int kMulticastPort = 30201;
constexpr int kReceiveAckPort = 40001;
constexpr int kReceiveTimeoutUs = 200000;
constexpr float kAckPollMs = 10.0f;

std::atomic<bool> stop_requested(false);

void handle_sig(int)
{
    stop_requested = true;
}

bool unpack_srv_state(const std::uint8_t* buffer, std::size_t size, SrvState& out)
{
    if (!serialization::unpackMessage(buffer, size, out, getClientHmacKey)) {
        std::cerr << "Failed to unpack SrvState" << std::endl;
        return false;
    }
    return true;
}

bool pack_core_request(const CoreRequestVariantPtr& request, std::uint8_t* buffer, std::size_t buffer_size, std::size_t& written_size)
{
    if (!serialization::packMessage(request, buffer, buffer_size, written_size, getClientHmacKey)) {
        std::cerr << "Failed to pack CoreRequestVariantPtr" << std::endl;
        return false;
    }
    return true;
}

bool unpack_core_response(const std::uint8_t* buffer, std::size_t size, CoreResponseVariantPtr& out)
{
    if (!serialization::unpackMessage(buffer, size, out, getClientHmacKey)) {
        std::cerr << "Failed to unpack CoreResponseVariantPtr" << std::endl;
        return false;
    }
    return true;
}

uint32_t perform_handshake(McastClient<SrvState, CoreRequestVariantPtr, CoreResponseVariantPtr>& client) {
    constexpr uint16_t handshake_seq = 1;
    constexpr int kMaxAttempts = 3;
    
    SdkHandshakeReq handshake{};
    handshake.client_id = kClientId;
    handshake.sequence_id = handshake_seq;
    
    for (int attempt = 1; attempt <= kMaxAttempts; ++attempt) {
        try {
            handshake.timestamp_us = static_cast<uint64_t>(get_time_now());
            CoreRequestVariantPtr request = std::make_unique<CoreRequestVariantPtr::element_type>(handshake);
            client.send(request);

            CoreResponseVariantPtr ack = client.waitAck(kClientId, handshake_seq, 3000.0f);
            if (!ack) {
                throw std::runtime_error("Handshake timed out");
            }

            if (!std::holds_alternative<SdkHandshakeRes>(*ack)) {
                throw std::runtime_error("Unexpected handshake response type");
            }

            const auto& handshake_res = std::get<SdkHandshakeRes>(*ack);
            if (handshake_res.payload.status != CommandResponseStatus::kSuccess) {
                throw std::runtime_error("Handshake rejected, status: " + enumToString(handshake_res.payload.status));
            }

            std::cout << "Client 10001 handshake success on attempt " << attempt 
                      << ", session_id=" << handshake_res.assigned_session_id << std::endl;
            
            return handshake_res.assigned_session_id;

        } catch (const std::runtime_error& e) {
            std::cerr << "Attempt " << attempt << " failed: " << e.what() << std::endl;
            if (attempt == kMaxAttempts) {
                throw std::runtime_error("Handshake failed after 10 attempts for client 10001");
            }
            // Optional: Add std::this_thread::sleep_for here to back off before retrying
        }
    }
    throw std::runtime_error("Handshake failed for client 10001");
}

void send_demo_config(McastClient<SrvState, CoreRequestVariantPtr, CoreResponseVariantPtr>& client, uint32_t session_id, uint32_t sequence_id, bool read_only)
{
    SdkConfigReq config{};
    config.client_id = kClientId;
    config.session_id = session_id;
    config.sequence_id = sequence_id;
    config.timestamp_us = static_cast<uint64_t>(get_time_now());
    config.payload.read_only = read_only;
    CoreRequestVariantPtr request = std::make_unique<CoreRequestVariantPtr::element_type>(config);
    client.send(request);

    CoreResponseVariantPtr ack = client.waitAck(kClientId, sequence_id, 3000.0f);
    if (!ack) {
        throw std::runtime_error("Config ack timed out for client 10001");
    }

    if (!std::holds_alternative<SdkConfigRes>(*ack)) {
        throw std::runtime_error("Unexpected config response type");
    }

    const auto& config_res = std::get<SdkConfigRes>(*ack);
    std::cout << "Client 10001 config response status="
              << enumToString(config_res.payload.status) << std::endl;
}

void send_demo_command(McastClient<SrvState, CoreRequestVariantPtr, CoreResponseVariantPtr>& client, uint32_t session_id, uint32_t sequence_id)
{
    SdkCommandReq command{};
    command.client_id = kClientId;
    command.session_id = session_id;
    command.sequence_id = sequence_id;
    command.timestamp_us = static_cast<uint64_t>(get_time_now());
    std::snprintf(command.payload.robot_name, sizeof(command.payload.robot_name), "test_robot");
    command.payload.arm_size = 1;
    command.payload.gripper_size = 1;
    command.payload.arm_joint_size[0] = 7;
    command.payload.gripper_joint_size[0] = 1;
    command.payload.enable_jog = 0;
    command.payload.activated_control_strategy = ControlStrategy::kJoint;
    command.payload.target_count = 1;

    CoreRequestVariantPtr request = std::make_unique<CoreRequestVariantPtr::element_type>(command);
    client.send(request);

    CoreResponseVariantPtr ack = client.waitAck(kClientId, sequence_id, 3000.0f);
    if (!ack) {
        throw std::runtime_error("Command ack timed out for client 10001");
    }

    if (!std::holds_alternative<SdkCommandRes>(*ack)) {
        throw std::runtime_error("Unexpected command response type");
    }

    const auto& command_res = std::get<SdkCommandRes>(*ack);
    std::cout << "Client 10001 command response status="
              << enumToString(command_res.payload.status) << std::endl;
}

}  // namespace

int main()
{
    std::signal(SIGINT, handle_sig);
    int sequence_id = 1;

    try {
#ifdef PROJECT_ROOT_DIR
        const std::string keys_path = std::string(PROJECT_ROOT_DIR) + "/cuarm_rt_control/src/keys.json";
#else
        const std::string keys_path = "keys.json";
#endif
        if (!loadClientKeys(keys_path)) {
            throw std::runtime_error("Failed to load client keys from " + keys_path);
        }

        McastClient<SrvState, CoreRequestVariantPtr, CoreResponseVariantPtr> client(
            kServerIp,
            kServerPort,
            kMulticastIp,
            kMulticastPort,
            kReceiveAckPort,
            unpack_srv_state,
            pack_core_request,
            unpack_core_response,
            kAckPollMs);

        std::cout << "Client 10001 connected to server " << kServerIp << ":" << kServerPort
                  << ", multicast " << kMulticastIp << ":" << kMulticastPort << std::endl;

        std::cout << "Perform handshake." << std::endl;
        const uint32_t session_id = perform_handshake(client);
        std::cout << "--------------------------------" << std::endl;
        std::cout << "Send demo command." << std::endl;
        send_demo_command(client, session_id, sequence_id++);
        std::cout << "--------------------------------" << std::endl;
        std::cout << "Send demo config, read_only=true." << std::endl;
        send_demo_config(client, session_id, sequence_id++, true);
        std::cout << "--------------------------------" << std::endl;
        std::cout << "Send demo config, read_only=false." << std::endl;
        send_demo_config(client, session_id, sequence_id++, false);
        std::cout << "--------------------------------" << std::endl;
        std::cout << "Send demo command." << std::endl;
        send_demo_command(client, session_id, sequence_id++);
        std::cout << "--------------------------------" << std::endl;

        while (!stop_requested) {
            send_demo_command(client, session_id, sequence_id++);

            SrvState state{};
            if (client.receive(state, kReceiveTimeoutUs)) {
                std::cout << "Client 10001 received SrvState seq=" << state.sequence_id
                          << " system_state=" << enumToString(state.payload.system_state) << std::endl;
            }
        }

        client.close();
        std::cout << "Client 10001 closed." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Client 10001 error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
