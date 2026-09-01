#include "platform_crypto.h"
#include "platform_serialization.h"
#include "platform_state.h"

#include <array>
#include <cstdint>
#include <iostream>
#include <string>

namespace {

using robot::platform::CoreRequestVariantPtr;
using robot::platform::CoreResponseVariantPtr;
using robot::platform::SdkHandshakeReq;
using robot::platform::SdkHandshakeRes;

const std::string* testKey(std::uint16_t) {
    static const std::string key = "0123456789abcdef";
    return &key;
}

bool appendTestHmac(std::uint8_t* buffer, std::size_t payload_size, std::size_t& packet_size) {
    std::uint8_t hmac[HMAC_KEY_SIZE]{};
    const auto* key = reinterpret_cast<const std::uint8_t*>(testKey(0)->data());
    return robot::platform::crypto::sendBufferAppendHmac(
        buffer, payload_size, key, HMAC_KEY_SIZE, hmac, packet_size);
}

}  // namespace

int main() {
    constexpr std::size_t kBufferSize = 1024;
    std::array<std::uint8_t, kBufferSize> body{};

    SdkHandshakeReq handshake{};
    handshake.client_id = 10003;
    handshake.sequence_id = 42;
    handshake.telemetry_port = 9988;
    handshake.timestamp_us = 123456;
    handshake.use_flatbuffers = true;
    handshake.protocol.extension_present = true;
    handshake.protocol.max_major = robot::platform::Protocol::kCurrentMajor;
    handshake.protocol.capabilities =
        robot::platform::Protocol::Capability::kFlatBuffersPayload
        | robot::platform::Protocol::Capability::kAppendOnlyTrailingFields;

    std::size_t payload_size = 0;
    if (!robot::platform::serialization::toBytes(
            handshake, body.data(), body.size() - HMAC_KEY_SIZE, payload_size)) {
        std::cerr << "failed to serialize FlatBuffers handshake request\n";
        return 1;
    }

    SdkHandshakeReq decoded{};
    if (!robot::platform::serialization::fromBytes(body.data(), payload_size, decoded)
        || !decoded.use_flatbuffers
        || decoded.client_id != handshake.client_id
        || decoded.sequence_id != handshake.sequence_id
        || decoded.telemetry_port != handshake.telemetry_port
        || decoded.timestamp_us != handshake.timestamp_us
        || !decoded.protocol.extension_present
        || (decoded.protocol.capabilities
            & robot::platform::Protocol::Capability::kFlatBuffersPayload) == 0) {
        std::cerr << "failed to decode FlatBuffers handshake request\n";
        return 2;
    }

    std::size_t packet_size = 0;
    if (!appendTestHmac(body.data(), payload_size, packet_size)) {
        std::cerr << "failed to append FlatBuffers handshake HMAC\n";
        return 3;
    }

    CoreRequestVariantPtr request;
    if (!robot::platform::serialization::unpackMessage(
            body.data(), packet_size, request, testKey)
        || !request
        || !std::holds_alternative<SdkHandshakeReq>(*request)) {
        std::cerr << "FlatBuffers handshake request failed unpackMessage\n";
        return 4;
    }

    SdkHandshakeRes response{};
    response.request_client_id = handshake.client_id;
    response.request_sequence_id = handshake.sequence_id;
    response.request_received_us = 1000;
    response.response_sent_us = 2000;
    response.assigned_session_id = 4242;
    response.payload.status = robot::platform::CommandResponseStatus::kSuccess;
    std::snprintf(response.payload.robot_name, MAX_NAME_SIZE, "%s", "spark2");
    response.include_robot_name = true;
    response.use_flatbuffers = true;
    response.protocol.extension_present = true;
    response.protocol.major = robot::platform::Protocol::kCurrentMajor;
    response.protocol.capabilities =
        robot::platform::Protocol::Capability::kFlatBuffersPayload
        | robot::platform::Protocol::Capability::kAppendOnlyTrailingFields;

    std::size_t response_payload_size = 0;
    if (!robot::platform::serialization::toBytes(
            response, body.data(), body.size() - HMAC_KEY_SIZE, response_payload_size)) {
        std::cerr << "failed to serialize FlatBuffers handshake response\n";
        return 5;
    }

    SdkHandshakeRes decoded_response{};
    if (!robot::platform::serialization::fromBytes(
            body.data(), response_payload_size, decoded_response)
        || !decoded_response.use_flatbuffers
        || decoded_response.request_client_id != response.request_client_id
        || decoded_response.assigned_session_id != response.assigned_session_id
        || decoded_response.payload.status != response.payload.status
        || std::string(decoded_response.payload.robot_name) != "spark2") {
        std::cerr << "failed to decode FlatBuffers handshake response\n";
        return 6;
    }

    std::cout << "FlatBuffers handshake tests passed\n";
    return 0;
}
