#include "platform_crypto.h"
#include "platform_serialization.h"
#include "platform_state.h"
#include "protocol_extensions.h"

#include <array>
#include <cstdint>
#include <iostream>
#include <string>

namespace {

using robot::platform::CoreRequestVariantPtr;
using robot::platform::SdkHandshakeReq;

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

bool unpack(const std::uint8_t* buffer, std::size_t size, CoreRequestVariantPtr& value) {
    return robot::platform::serialization::unpackMessage(buffer, size, value, testKey);
}

}  // namespace

int main() {
    constexpr std::size_t kBufferSize = 512;
    std::array<std::uint8_t, kBufferSize> body{};

    SdkHandshakeReq handshake{};
    handshake.client_id = 10003;
    handshake.sequence_id = 42;
    handshake.telemetry_port = 9988;
    handshake.timestamp_us = 123456;

    std::size_t base_size = 0;
    if (!robot::platform::serialization::toBytes(
            handshake, body.data(), body.size() - HMAC_KEY_SIZE, base_size)) {
        std::cerr << "failed to serialize base handshake\n";
        return 1;
    }
    constexpr std::size_t kLegacyHandshakeSize =
        sizeof(std::uint32_t) + sizeof(robot::platform::MessageType)
        + sizeof(std::uint16_t) + sizeof(std::uint32_t)
        + sizeof(std::uint16_t) + sizeof(std::uint64_t);
    if (base_size != kLegacyHandshakeSize) {
        std::cerr << "legacy handshake wire layout changed\n";
        return 2;
    }

    // A reader built from this compatibility release must accept authenticated
    // fields appended by a newer peer.
    std::array<std::uint8_t, kBufferSize> extended = body;
    constexpr std::uint16_t kTestFieldId = 0x9001;
    const std::uint32_t extension_value = 0x01020304;
    std::size_t extended_body_size = base_size;
    if (!robot::platform::protocol::appendExtensionField(
            extended.data(), extended.size() - HMAC_KEY_SIZE, extended_body_size,
            kTestFieldId, extension_value)) {
        std::cerr << "failed to append extension field\n";
        return 3;
    }
    std::size_t extended_packet_size = 0;
    if (!appendTestHmac(
            extended.data(), extended_body_size, extended_packet_size)) {
        std::cerr << "failed to append extension HMAC\n";
        return 4;
    }

    CoreRequestVariantPtr decoded;
    if (!unpack(extended.data(), extended_packet_size, decoded)
        || !decoded
        || !std::holds_alternative<SdkHandshakeReq>(*decoded)) {
        std::cerr << "authenticated trailing extension was rejected\n";
        return 5;
    }
    std::uint32_t decoded_extension = 0;
    if (!robot::platform::protocol::readExtensionField(
            extended.data(), extended_body_size, base_size,
            kTestFieldId, decoded_extension)
        || decoded_extension != extension_value) {
        std::cerr << "failed to read extension field\n";
        return 6;
    }

    // Negotiation is an optional authenticated suffix. It does not alter the
    // legacy prefix and is decoded by compatibility-aware peers.
    SdkHandshakeReq negotiated = handshake;
    negotiated.protocol.extension_present = true;
    std::size_t negotiated_size = 0;
    if (!robot::platform::serialization::toBytes(
            negotiated, body.data(), body.size() - HMAC_KEY_SIZE, negotiated_size)
        || negotiated_size <= base_size) {
        std::cerr << "failed to serialize protocol negotiation\n";
        return 7;
    }
    SdkHandshakeReq decoded_negotiation{};
    if (!robot::platform::serialization::fromBytes(
            body.data(), negotiated_size, decoded_negotiation)
        || !decoded_negotiation.protocol.extension_present
        || decoded_negotiation.protocol.max_major
            != robot::platform::Protocol::kCurrentMajor
        || (decoded_negotiation.protocol.capabilities
            & robot::platform::Protocol::Capability::kAppendOnlyTrailingFields) == 0) {
        std::cerr << "failed to decode protocol negotiation\n";
        return 8;
    }

    // A shorter payload with a valid HMAC must fail safely instead of reading
    // beyond the received datagram.
    std::array<std::uint8_t, kBufferSize> truncated = body;
    std::size_t truncated_packet_size = 0;
    if (!appendTestHmac(truncated.data(), base_size - 1, truncated_packet_size)) {
        std::cerr << "failed to append truncated-payload HMAC\n";
        return 9;
    }
    decoded.reset();
    if (unpack(truncated.data(), truncated_packet_size, decoded)) {
        std::cerr << "truncated payload was unexpectedly accepted\n";
        return 10;
    }

    // Unknown extension bytes remain covered by HMAC.
    extended[base_size] ^= 0xff;
    decoded.reset();
    if (unpack(extended.data(), extended_packet_size, decoded)) {
        std::cerr << "tampered trailing extension was unexpectedly accepted\n";
        return 11;
    }

    std::cout << "protocol compatibility tests passed\n";
    return 0;
}
