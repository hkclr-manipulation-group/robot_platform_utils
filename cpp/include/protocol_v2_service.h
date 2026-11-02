#pragma once
#include "protocol_v2.h"

namespace robot::platform::protocol_v2 {
struct Reply {
    std::uint32_t status = 0;
    std::vector<std::uint8_t> body;
};
// Pure, read-only service core shared by RT and the loopback interoperability tests.
inline Reply describe(const std::uint8_t* data, std::size_t size, Profile profile) {
    if (size > kMaxMessage) return {413, {}};
    Hello hello;
    auto error = decode(data, size, hello);
    if (error == Error::Ok) error = negotiate(hello, profile);
    switch (error) {
        case Error::UnsupportedVersion: return {406, {}};
        case Error::UnsupportedRequiredField: return {422, {}};
        case Error::MissingCapability: return {412, {}};
        case Error::Malformed: return {400, {}};
        default: break;
    }
    Reply reply;
    if (!encode(profile, reply.body)) reply.status = 503;
    return reply;
}
// Revision is an opaque topology fingerprint, scoped by boot identity, not time.
inline std::uint64_t topologyRevision(const std::vector<Component>& components) {
    std::uint64_t hash = 14695981039346656037ull;
    const auto mix = [&](std::uint8_t byte) { hash = (hash ^ byte) * 1099511628211ull; };
    for (const auto& c : components) {
        Writer record;
        if (!encodeComponent(c, record)) return 0;
        for (auto byte : record.data) mix(byte);
    }
    return hash ? hash : 1;
}
} // namespace robot::platform::protocol_v2
