#include "platform_flatbuffers.h"

#include <cstdio>
#include <cstring>
#include <string>
#include <type_traits>

#include <flatbuffers/flatbuffers.h>

#include "handshake_generated.h"

namespace robot::platform::flatbuffers_codec {
namespace {

using fb::CommandResponseStatus;
using fb::NegotiatedProtocol;
using fb::ProtocolNegotiation;
using fb::SdkHandshakeReqBody;
using fb::SdkHandshakeResBody;

constexpr std::size_t kFixedHeaderSize =
    sizeof(std::uint32_t) + sizeof(MessageType) + sizeof(std::uint16_t);
constexpr std::size_t kFlatBufferMarkerSize = sizeof(std::uint32_t);

CommandResponseStatus toFbStatus(robot::platform::CommandResponseStatus status) {
    return static_cast<CommandResponseStatus>(
        static_cast<std::underlying_type_t<robot::platform::CommandResponseStatus>>(status));
}

robot::platform::CommandResponseStatus fromFbStatus(CommandResponseStatus status) {
    return static_cast<robot::platform::CommandResponseStatus>(
        static_cast<std::uint8_t>(status));
}

flatbuffers::Offset<ProtocolNegotiation> buildProtocolNegotiation(
    flatbuffers::FlatBufferBuilder& builder,
    const robot::platform::ProtocolNegotiation& protocol) {
    fb::ProtocolNegotiationBuilder negotiation_builder(builder);
    negotiation_builder.add_extension_present(protocol.extension_present);
    negotiation_builder.add_min_major(protocol.min_major);
    negotiation_builder.add_max_major(protocol.max_major);
    negotiation_builder.add_max_minor(protocol.max_minor);
    negotiation_builder.add_capabilities(protocol.capabilities);
    return negotiation_builder.Finish();
}

flatbuffers::Offset<NegotiatedProtocol> buildNegotiatedProtocol(
    flatbuffers::FlatBufferBuilder& builder,
    const robot::platform::NegotiatedProtocol& protocol) {
    fb::NegotiatedProtocolBuilder negotiation_builder(builder);
    negotiation_builder.add_extension_present(protocol.extension_present);
    negotiation_builder.add_major(protocol.major);
    negotiation_builder.add_minor(protocol.minor);
    negotiation_builder.add_capabilities(protocol.capabilities);
    return negotiation_builder.Finish();
}

void readProtocolNegotiation(
    const ProtocolNegotiation* protocol,
    robot::platform::ProtocolNegotiation& out) {
    out = robot::platform::ProtocolNegotiation{};
    if (protocol == nullptr) {
        return;
    }
    out.extension_present = protocol->extension_present();
    out.min_major = protocol->min_major();
    out.max_major = protocol->max_major();
    out.max_minor = protocol->max_minor();
    out.capabilities = protocol->capabilities();
}

void readNegotiatedProtocol(
    const NegotiatedProtocol* protocol,
    robot::platform::NegotiatedProtocol& out) {
    out = robot::platform::NegotiatedProtocol{};
    if (protocol == nullptr) {
        return;
    }
    out.extension_present = protocol->extension_present();
    out.major = protocol->major();
    out.minor = protocol->minor();
    out.capabilities = protocol->capabilities();
}

bool verifyFlatBuffer(const std::uint8_t* flatbuffer, std::size_t flatbuffer_size) {
    flatbuffers::Verifier verifier(flatbuffer, flatbuffer_size);
    return verifier.VerifyBuffer<SdkHandshakeReqBody>(nullptr)
        || verifier.VerifyBuffer<SdkHandshakeResBody>(nullptr);
}

}  // namespace

bool isFlatBufferPayload(const std::uint8_t* cursor, std::size_t remaining) {
    if (cursor == nullptr || remaining < sizeof(std::uint32_t)) {
        return false;
    }
    std::uint32_t marker = 0;
    std::memcpy(&marker, cursor, sizeof(marker));
    return marker == kFlatBufferPayloadMagic;
}

bool encodeSdkHandshakeReq(
    const SdkHandshakeReq& value,
    std::uint8_t* buffer,
    std::size_t buffer_size,
    std::size_t& written_size) {
    written_size = 0;
    if (buffer == nullptr || buffer_size < kFixedHeaderSize + kFlatBufferMarkerSize) {
        return false;
    }

    flatbuffers::FlatBufferBuilder builder(256);
    const auto protocol_offset = buildProtocolNegotiation(builder, value.protocol);
    fb::SdkHandshakeReqBodyBuilder body_builder(builder);
    body_builder.add_sequence_id(value.sequence_id);
    body_builder.add_telemetry_port(value.telemetry_port);
    body_builder.add_timestamp_us(value.timestamp_us);
    body_builder.add_protocol(protocol_offset);
    const auto body_offset = body_builder.Finish();
    builder.Finish(body_offset);

    const std::size_t payload_capacity =
        buffer_size - kFixedHeaderSize - kFlatBufferMarkerSize;
    if (builder.GetSize() > payload_capacity) {
        return false;
    }

    std::uint8_t* cursor = buffer;
    std::memcpy(cursor, &value.magic_header, sizeof(value.magic_header));
    cursor += sizeof(value.magic_header);
    const MessageType message_type = MessageType::kSdkHandshakeReq;
    std::memcpy(cursor, &message_type, sizeof(message_type));
    cursor += sizeof(message_type);
    std::memcpy(cursor, &value.client_id, sizeof(value.client_id));
    cursor += sizeof(value.client_id);
    const std::uint32_t marker = kFlatBufferPayloadMagic;
    std::memcpy(cursor, &marker, sizeof(marker));
    cursor += sizeof(marker);
    std::memcpy(cursor, builder.GetBufferPointer(), builder.GetSize());
    written_size = kFixedHeaderSize + kFlatBufferMarkerSize + builder.GetSize();
    return true;
}

bool decodeSdkHandshakeReq(
    const std::uint8_t* buffer,
    std::size_t buffer_size,
    SdkHandshakeReq& value) {
    if (buffer == nullptr || buffer_size < kFixedHeaderSize + kFlatBufferMarkerSize) {
        return false;
    }

    const std::uint8_t* cursor = buffer;
    std::memcpy(&value.magic_header, cursor, sizeof(value.magic_header));
    cursor += sizeof(value.magic_header);
    cursor += sizeof(MessageType);
    std::memcpy(&value.client_id, cursor, sizeof(value.client_id));
    cursor += sizeof(value.client_id);

    if (!isFlatBufferPayload(cursor, buffer_size - static_cast<std::size_t>(cursor - buffer))) {
        return false;
    }
    cursor += sizeof(std::uint32_t);

    const std::size_t flatbuffer_size =
        buffer_size - static_cast<std::size_t>(cursor - buffer);
    if (!verifyFlatBuffer(cursor, flatbuffer_size)) {
        return false;
    }

    const auto* body = flatbuffers::GetRoot<SdkHandshakeReqBody>(cursor);
    if (body == nullptr) {
        return false;
    }

    value.sequence_id = body->sequence_id();
    value.telemetry_port = body->telemetry_port();
    value.timestamp_us = body->timestamp_us();
    readProtocolNegotiation(body->protocol(), value.protocol);
    value.use_flatbuffers = true;
    return true;
}

bool encodeSdkHandshakeRes(
    const SdkHandshakeRes& value,
    std::uint8_t* buffer,
    std::size_t buffer_size,
    std::size_t& written_size) {
    written_size = 0;
    if (buffer == nullptr || buffer_size < kFixedHeaderSize + kFlatBufferMarkerSize) {
        return false;
    }

    flatbuffers::FlatBufferBuilder builder(512);
    const auto protocol_offset = buildNegotiatedProtocol(builder, value.protocol);
    const flatbuffers::Offset<flatbuffers::String> robot_name_offset =
        value.include_robot_name
            ? builder.CreateString(value.payload.robot_name)
            : 0;
    fb::SdkHandshakeResBodyBuilder body_builder(builder);
    body_builder.add_request_sequence_id(value.request_sequence_id);
    body_builder.add_request_received_us(value.request_received_us);
    body_builder.add_response_sent_us(value.response_sent_us);
    body_builder.add_assigned_session_id(value.assigned_session_id);
    body_builder.add_status(toFbStatus(value.payload.status));
    body_builder.add_robot_name(robot_name_offset);
    body_builder.add_protocol(protocol_offset);
    const auto body_offset = body_builder.Finish();
    builder.Finish(body_offset);

    const std::size_t payload_capacity =
        buffer_size - kFixedHeaderSize - kFlatBufferMarkerSize;
    if (builder.GetSize() > payload_capacity) {
        return false;
    }

    std::uint8_t* cursor = buffer;
    std::memcpy(cursor, &value.magic_header, sizeof(value.magic_header));
    cursor += sizeof(value.magic_header);
    const MessageType message_type = MessageType::kSdkHandshakeRes;
    std::memcpy(cursor, &message_type, sizeof(message_type));
    cursor += sizeof(message_type);
    std::memcpy(cursor, &value.request_client_id, sizeof(value.request_client_id));
    cursor += sizeof(value.request_client_id);
    const std::uint32_t marker = kFlatBufferPayloadMagic;
    std::memcpy(cursor, &marker, sizeof(marker));
    cursor += sizeof(marker);
    std::memcpy(cursor, builder.GetBufferPointer(), builder.GetSize());
    written_size = kFixedHeaderSize + kFlatBufferMarkerSize + builder.GetSize();
    return true;
}

bool decodeSdkHandshakeRes(
    const std::uint8_t* buffer,
    std::size_t buffer_size,
    SdkHandshakeRes& value) {
    if (buffer == nullptr || buffer_size < kFixedHeaderSize + kFlatBufferMarkerSize) {
        return false;
    }

    const std::uint8_t* cursor = buffer;
    std::memcpy(&value.magic_header, cursor, sizeof(value.magic_header));
    cursor += sizeof(value.magic_header);
    cursor += sizeof(MessageType);
    std::memcpy(&value.request_client_id, cursor, sizeof(value.request_client_id));
    cursor += sizeof(value.request_client_id);

    if (!isFlatBufferPayload(cursor, buffer_size - static_cast<std::size_t>(cursor - buffer))) {
        return false;
    }
    cursor += sizeof(std::uint32_t);

    const std::size_t flatbuffer_size =
        buffer_size - static_cast<std::size_t>(cursor - buffer);
    if (!verifyFlatBuffer(cursor, flatbuffer_size)) {
        return false;
    }

    const auto* body = flatbuffers::GetRoot<SdkHandshakeResBody>(cursor);
    if (body == nullptr) {
        return false;
    }

    value.request_sequence_id = body->request_sequence_id();
    value.request_received_us = body->request_received_us();
    value.response_sent_us = body->response_sent_us();
    value.assigned_session_id = body->assigned_session_id();
    value.payload.status = fromFbStatus(body->status());
    std::memset(value.payload.robot_name, 0, MAX_NAME_SIZE);
    value.include_robot_name = false;
    if (const auto* robot_name = body->robot_name()) {
        std::snprintf(
            value.payload.robot_name,
            MAX_NAME_SIZE,
            "%s",
            robot_name->c_str());
        value.include_robot_name = true;
    }
    readNegotiatedProtocol(body->protocol(), value.protocol);
    value.use_flatbuffers = true;
    return true;
}

}  // namespace robot::platform::flatbuffers_codec
