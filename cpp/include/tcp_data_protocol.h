#ifndef TCP_DATA_PROTOCOL_H
#define TCP_DATA_PROTOCOL_H

#include <cstdint>

namespace robot::platform::tcp_data {

/** Magic for cold-path TCP application messages ("SP2D"). */
constexpr std::uint32_t kMagic = 0x53503244u;
constexpr std::uint16_t kVersion = 1u;

enum class MessageKind : std::uint16_t {
    kPing = 1,
    kPong = 2,
    kBlobBegin = 10,
    kBlobChunk = 11,
    kBlobEnd = 12,
    kAck = 100,
    kError = 101,
};

#pragma pack(push, 1)
struct MessageHeader {
    std::uint32_t magic = kMagic;
    std::uint16_t version = kVersion;
    std::uint16_t kind = 0;
    std::uint32_t sequence = 0;
    std::uint32_t payload_size = 0;
};
#pragma pack(pop)

static_assert(sizeof(MessageHeader) == 16, "TcpData MessageHeader must be 16 bytes");

constexpr std::uint32_t kDefaultChunkSize = 64u * 1024u;

}  // namespace robot::platform::tcp_data

#endif
