#ifndef PLATFORM_FLATBUFFERS_H
#define PLATFORM_FLATBUFFERS_H

#include <cstddef>
#include <cstdint>

#include "platform_state.h"

namespace robot::platform::flatbuffers_codec {

constexpr std::uint32_t kFlatBufferPayloadMagic = 0x42454642U; // "FBFB"

bool isFlatBufferPayload(const std::uint8_t* cursor, std::size_t remaining);

bool encodeSdkHandshakeReq(
    const SdkHandshakeReq& value,
    std::uint8_t* buffer,
    std::size_t buffer_size,
    std::size_t& written_size);

bool decodeSdkHandshakeReq(
    const std::uint8_t* buffer,
    std::size_t buffer_size,
    SdkHandshakeReq& value);

bool encodeSdkHandshakeRes(
    const SdkHandshakeRes& value,
    std::uint8_t* buffer,
    std::size_t buffer_size,
    std::size_t& written_size);

bool decodeSdkHandshakeRes(
    const std::uint8_t* buffer,
    std::size_t buffer_size,
    SdkHandshakeRes& value);

}  // namespace robot::platform::flatbuffers_codec

#endif
