#ifndef TCP_DATA_PROTOCOL_H
#define TCP_DATA_PROTOCOL_H

#include <cstdint>

namespace robot::platform::tcp_data {

/** Magic for TCP data-channel application messages ("SP2D"). */
constexpr std::uint32_t kMagic = 0x53503244u;
constexpr std::uint16_t kVersion = 2u;

enum class MessageKind : std::uint16_t {
    kPing = 1,
    kPong = 2,

    kBlobBegin = 10,
    kBlobChunk = 11,
    kBlobEnd = 12,

    kRpcRequest = 20,
    kRpcResponse = 21,

    kBlobDownloadBegin = 30,
    kBlobDownloadChunk = 31,
    kBlobDownloadEnd = 32,

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

struct RpcRequestHeader {
    std::uint32_t service_id = 0;
    std::uint32_t method_id = 0;
};

struct RpcResponseHeader {
    std::uint32_t status_code = 0;
};
#pragma pack(pop)

static_assert(sizeof(MessageHeader) == 16, "TcpData MessageHeader must be 16 bytes");
static_assert(sizeof(RpcRequestHeader) == 8, "RpcRequestHeader must be 8 bytes");
static_assert(sizeof(RpcResponseHeader) == 4, "RpcResponseHeader must be 4 bytes");

constexpr std::uint32_t kDefaultChunkSize = 64u * 1024u;
constexpr std::uint32_t kRpcHeaderSize = sizeof(RpcRequestHeader);

}  // namespace robot::platform::tcp_data

#endif
