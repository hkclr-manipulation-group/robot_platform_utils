#pragma once

#include "platform_state.h"
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

namespace robot::platform::tcp_data::teaching {
// Teach service (2), method 1. Existing UDP layouts and service IDs are unchanged.
inline constexpr uint32_t kReadFrozenTrajectory = 1;
inline constexpr uint32_t kVersion = 1;
inline constexpr uint32_t kMaxChunkSamples = 256;
enum class ReadStatus { kOk, kInvalid, kRecording, kEmpty, kChanged };
struct Request {
    uint32_t arm = 0;
    uint64_t recording = 0; // 0 only for the initial metadata query
    uint32_t offset = 0, limit = 0; // limit=0: metadata only
};
struct Chunk {
    uint32_t arm = 0;
    uint64_t recording = 0;
    uint32_t offset = 0, total = 0, joints = 0;
    double period_sec = 0;
    std::vector<double> positions_rad; // sample-major, joint-major
};
namespace codec {
inline void put(std::vector<uint8_t>& b, uint64_t v, int n) {
    for (int i=0;i<n;++i) b.push_back(uint8_t(v>>(8*i)));
}
inline uint64_t get(const std::vector<uint8_t>& b, size_t& at, int n) {
    uint64_t v=0; for (int i=0;i<n;++i) v|=uint64_t(b[at++])<<(8*i); return v;
}
inline void putDouble(std::vector<uint8_t>& b, double d) {
    static_assert(sizeof(double)==8, "trajectory format requires binary64");
    uint64_t v; std::memcpy(&v,&d,8); put(b,v,8);
}
inline double getDouble(const std::vector<uint8_t>& b, size_t& at) {
    auto v=get(b,at,8); double d; std::memcpy(&d,&v,8); return d;
}
}
inline bool valid(const Request& r) {
    return r.arm<MAX_ARM_SIZE && r.limit<=kMaxChunkSamples &&
        (r.recording!=0 || (r.offset==0 && r.limit==0));
}
inline std::vector<uint8_t> encode(const Request& r) {
    if (!valid(r)) return {};
    std::vector<uint8_t> b;
    codec::put(b,kVersion,4); codec::put(b,r.arm,4); codec::put(b,r.recording,8);
    codec::put(b,r.offset,4); codec::put(b,r.limit,4); return b;
}
inline bool decode(const std::vector<uint8_t>& b, Request& out) {
    if (b.size()!=24) return false;
    size_t at=0; if (codec::get(b,at,4)!=kVersion) return false;
    Request r; r.arm=uint32_t(codec::get(b,at,4)); r.recording=codec::get(b,at,8);
    r.offset=uint32_t(codec::get(b,at,4)); r.limit=uint32_t(codec::get(b,at,4));
    if (!valid(r)) return false;
    out=r; return true;
}
inline bool valid(const Chunk& c) {
    if (c.arm>=MAX_ARM_SIZE || !c.recording || !c.total || !c.joints ||
        c.joints>MAX_ARM_JOINT_SIZE || c.offset>c.total ||
        !std::isfinite(c.period_sec) || c.period_sec<=0 ||
        c.positions_rad.size()%c.joints!=0) return false;
    const auto count=c.positions_rad.size()/c.joints;
    if (count>kMaxChunkSamples || count>c.total-c.offset) return false;
    for (double q:c.positions_rad) if (!std::isfinite(q)) return false;
    return true;
}
inline std::vector<uint8_t> encode(const Chunk& c) {
    if (!valid(c)) return {};
    std::vector<uint8_t> b;
    codec::put(b,kVersion,4); codec::put(b,c.arm,4); codec::put(b,c.recording,8);
    codec::put(b,c.offset,4); codec::put(b,c.total,4); codec::put(b,c.joints,4);
    codec::putDouble(b,c.period_sec);
    codec::put(b,c.positions_rad.size()/c.joints,4);
    for (double q:c.positions_rad) codec::putDouble(b,q);
    return b;
}
inline bool decode(const std::vector<uint8_t>& b, Chunk& out) {
    if (b.size()<40) return false;
    size_t at=0; if (codec::get(b,at,4)!=kVersion) return false;
    Chunk c; c.arm=uint32_t(codec::get(b,at,4)); c.recording=codec::get(b,at,8);
    c.offset=uint32_t(codec::get(b,at,4)); c.total=uint32_t(codec::get(b,at,4));
    c.joints=uint32_t(codec::get(b,at,4)); c.period_sec=codec::getDouble(b,at);
    const auto count=codec::get(b,at,4);
    if (!c.joints || c.joints>MAX_ARM_JOINT_SIZE || count>kMaxChunkSamples ||
        b.size()-at!=count*c.joints*8) return false;
    c.positions_rad.reserve(size_t(count)*c.joints);
    while (at<b.size()) c.positions_rad.push_back(codec::getDouble(b,at));
    if (!valid(c)) return false;
    out=std::move(c); return true;
}
} // namespace robot::platform::tcp_data::teaching
