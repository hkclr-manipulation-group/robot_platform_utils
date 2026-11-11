#pragma once
#include "protocol_v2.h"
#include "platform_crypto.h"
#include <cmath>
#include <cstring>
#include <array>

namespace robot::platform::protocol_udp {
namespace base = protocol_v2;
constexpr std::uint32_t kMagic = 0x32555253; // SRU2, distinct from Legacy and TCP description.
constexpr std::size_t kHeaderSize = 80, kTagSize = 16, kMaxDatagram = 1200;
constexpr std::uint64_t kJointPosition = 1ull << 8, kOperationStatus = 1ull << 9;
constexpr std::uint64_t kFeatures = kJointPosition | kOperationStatus;
constexpr std::uint64_t kCartesianPose = 1ull << 10, kCartesianLine = 1ull << 11, kMixedSpace = 1ull << 12;
constexpr std::uint64_t kMixedFeatures = kFeatures | kCartesianPose | kCartesianLine | kMixedSpace;
enum class Kind : std::uint16_t { Submit = 1, Query = 2, Status = 3 };
enum class Phase : std::uint16_t { Received = 1, Accepted = 2, Running = 3, Succeeded = 4,
    Failed = 5, Superseded = 6, Expired = 7, Unknown = 8 };
enum class Reason : std::uint16_t { None = 0, Invalid = 1, Unsupported = 2, Revision = 3,
    Expired = 4, Ownership = 5, Fault = 6, Capacity = 7, Conflict = 8, NotFound = 9, Planner = 10 };
struct Context {
    std::uint16_t client = 0;
    std::uint32_t session = 0, sequence = 0;
    std::uint64_t command = 0, boot_high = 0, boot_low = 0, topology = 0, configuration = 0, issued_us = 0;
    std::uint32_t ttl_ms = 1000;
};
struct Target {
    std::uint32_t component = 0; std::vector<float> radians;
    std::uint16_t type = 1, frame = 0; // 1=Joint, 2=Cartesian pose, 3=Cartesian line; frame 0=model base/configured TCP.
    std::array<float,7> pose{}; // x,y,z in m, then unit quaternion w,x,y,z.
};
struct Command {
    // 0=speed-limited, 1=timed with independent velocity ceiling.
    std::uint16_t timing = 0;
    float velocity_scale = .2f, duration_sec = 0;
    std::vector<Target> targets;
};
struct Status { Phase phase = Phase::Received; Reason reason = Reason::None;
    std::uint32_t execution_sequence = 0; std::uint64_t diagnostics = 0; };
struct Packet { Kind kind = Kind::Submit; Context context; Command command; Status status; };
inline bool terminal(Phase p) { return p >= Phase::Succeeded; }
inline bool valid(const Context& c) {
    return c.client && c.session && c.sequence && c.command && (c.boot_high || c.boot_low) &&
        c.topology && c.configuration && c.issued_us && c.ttl_ms && c.ttl_ms <= 5000;
}
inline bool expired(const Context& c, std::uint64_t now) {
    return c.issued_us > now ? c.issued_us - now > 250000 : now - c.issued_us > std::uint64_t(c.ttl_ms) * 1000;
}
inline bool valid(const Command& c) {
    if (c.timing > 1 || !std::isfinite(c.velocity_scale) || c.velocity_scale <= 0 || c.velocity_scale > 1 ||
        !std::isfinite(c.duration_sec) || (c.timing == 0 ? c.duration_sec != 0 : c.duration_sec <= 0) ||
        c.targets.empty() || c.targets.size() > base::kMaxComponents) return false;
    for (std::size_t i = 0; i < c.targets.size(); ++i) {
        const auto& t = c.targets[i];
        if (!t.component || t.type<1 || t.type>3 || t.frame) return false;
        if (t.type==1) {
            if (t.radians.empty() || t.radians.size()>base::kMaxJoints) return false;
            for (float q:t.radians) if (!std::isfinite(q)) return false;
        } else {
            if (!t.radians.empty()) return false;
            for (float v:t.pose) if (!std::isfinite(v)) return false;
            double norm=0; for (int j=3;j<7;++j) norm+=double(t.pose[j])*t.pose[j];
            if (std::abs(norm-1)>1e-3) return false;
        }
        for (std::size_t j = 0; j < i; ++j) if (c.targets[j].component == t.component) return false;
    }
    return true;
}
inline bool putFloat(base::Writer& w, float v) {
    static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559, "IEEE754 float32 required");
    std::uint32_t bits; std::memcpy(&bits, &v, 4); return w.put(bits);
}
inline bool getFloat(base::Reader& r, float& v) {
    std::uint32_t bits; if (!r.get(bits)) return false; std::memcpy(&v, &bits, 4); return std::isfinite(v);
}
inline bool looksV2(const std::uint8_t* data, std::size_t size) {
    return data && size >= 4 && data[0] == 'S' && data[1] == 'R' && data[2] == 'U' && data[3] == '2';
}
inline bool encode(const Packet& p, std::vector<std::uint8_t>& out) {
    if (!valid(p.context)) return false;
    base::Writer body;
    if (p.kind == Kind::Submit) {
        if (!valid(p.command)) return false;
        base::Writer timing;
        if (!timing.put(p.command.timing) || !timing.put(std::uint16_t(0)) ||
            !putFloat(timing, p.command.velocity_scale) || !putFloat(timing, p.command.duration_sec) || !body.field(1, timing)) return false;
        for (const auto& t : p.command.targets) {
            base::Writer target;
            if (!target.put(t.component) || !target.put(t.type) || !target.put(std::uint16_t(t.type==1?t.radians.size():7))) return false;
            if (t.type==1) { for (float q:t.radians) if (!putFloat(target,q)) return false; }
            else {
                if (!target.put(t.frame) || !target.put(std::uint16_t(0))) return false;
                for (float v:t.pose) if (!putFloat(target,v)) return false;
            }
            if (!body.field(20, target)) return false;
        }
    } else if (p.kind == Kind::Status) {
        if (p.status.phase < Phase::Received || p.status.phase > Phase::Unknown || p.status.reason > Reason::Planner) return false;
        if (!body.put(static_cast<std::uint16_t>(p.status.phase)) || !body.put(static_cast<std::uint16_t>(p.status.reason)) ||
            !body.put(p.status.execution_sequence) || !body.put(p.status.diagnostics)) return false;
    } else if (p.kind != Kind::Query) return false;
    if (kHeaderSize + body.data.size() + kTagSize > kMaxDatagram) return false;
    const auto& c = p.context;
    base::Writer w;
    if (!w.put(kMagic) || !w.put(std::uint16_t(2)) || !w.put(static_cast<std::uint16_t>(p.kind)) ||
        !w.put(std::uint16_t(kHeaderSize)) || !w.put(std::uint16_t(0)) || !w.put(std::uint32_t(body.data.size())) ||
        !w.put(c.client) || !w.put(std::uint16_t(0)) || !w.put(c.session) || !w.put(c.sequence) || !w.put(c.command) ||
        !w.put(c.boot_high) || !w.put(c.boot_low) || !w.put(c.topology) || !w.put(c.configuration) ||
        !w.put(c.issued_us) || !w.put(c.ttl_ms) || !w.append(body.data.data(), body.data.size())) return false;
    out = std::move(w.data); return true;
}
// Decode plaintext only after the transport has verified the complete signed datagram.
inline bool decode(const std::uint8_t* data, std::size_t size, Packet& out) {
    if (!data || size < kHeaderSize || size + kTagSize > kMaxDatagram) return false;
    base::Reader r(data, size); Packet p;
    std::uint32_t magic, length; std::uint16_t version, kind, header, flags, reserved;
    auto& c = p.context;
    if (!r.get(magic) || !r.get(version) || !r.get(kind) || !r.get(header) || !r.get(flags) || !r.get(length) ||
        !r.get(c.client) || !r.get(reserved) || !r.get(c.session) || !r.get(c.sequence) || !r.get(c.command) ||
        !r.get(c.boot_high) || !r.get(c.boot_low) || !r.get(c.topology) || !r.get(c.configuration) ||
        !r.get(c.issued_us) || !r.get(c.ttl_ms) || magic != kMagic || version != 2 || header != kHeaderSize ||
        flags || reserved || length != r.remaining() || !valid(c)) return false;
    p.kind = static_cast<Kind>(kind);
    if (p.kind == Kind::Submit) {
        bool timing_seen = false; unsigned fields = 0;
        while (r.remaining()) {
            base::Field f;
            if (++fields > base::kMaxFields || !base::nextField(r, f)) return false;
            base::Reader value(f.data, f.size);
            if (f.tag == 1) {
                if (timing_seen || f.size != 12 || !value.get(p.command.timing) || !value.get(reserved) || reserved ||
                    !getFloat(value, p.command.velocity_scale) || !getFloat(value, p.command.duration_sec)) return false;
                timing_seen = true;
            } else if (f.tag == 20) {
                Target t; std::uint16_t count, target_reserved;
                if (p.command.targets.size()>=base::kMaxComponents || !value.get(t.component) || !value.get(t.type) ||
                    !value.get(count) || t.type<1 || t.type>3 || !count || count>base::kMaxJoints) return false;
                if (t.type==1) {
                    if (value.remaining()!=4u*count) return false;
                    t.radians.resize(count); for (auto& q:t.radians) if (!getFloat(value,q)) return false;
                } else {
                    if (count!=7 || !value.get(t.frame) || !value.get(target_reserved) || target_reserved || t.frame || value.remaining()!=28) return false;
                    for (auto& v:t.pose) if (!getFloat(value,v)) return false;
                }
                p.command.targets.push_back(std::move(t));
            } else if (f.flags & base::kCritical) return false;
        }
        if (!timing_seen || !valid(p.command)) return false;
    } else if (p.kind == Kind::Status) {
        std::uint16_t phase, reason;
        if (r.remaining() != 16 || !r.get(phase) || !r.get(reason) || !r.get(p.status.execution_sequence) ||
            !r.get(p.status.diagnostics) || phase < 1 || phase > 8 || reason > 10) return false;
        p.status.phase = static_cast<Phase>(phase); p.status.reason = static_cast<Reason>(reason);
    } else if (p.kind != Kind::Query || r.remaining()) return false;
    out = std::move(p); return true;
}
inline bool sign(const Packet& p, const std::string& key, std::vector<std::uint8_t>& out) {
    std::vector<std::uint8_t> bytes;
    // Preserve Legacy key material: the registry stores text and Legacy uses
    // its first HMAC_KEY_SIZE bytes, without hex decoding. Do not reinterpret it.
    if (key.size() < kTagSize || !encode(p, bytes)) return false;
    std::uint8_t tag[kTagSize];
    if (!crypto::computeHmacMd5(bytes.data(), bytes.size(), reinterpret_cast<const std::uint8_t*>(key.data()), kTagSize, tag)) return false;
    bytes.insert(bytes.end(), tag, tag + kTagSize); out = std::move(bytes); return true;
}
inline bool authenticate(const std::uint8_t* data, std::size_t size, const std::string& key, Packet& out) {
    if (!data || size < kHeaderSize + kTagSize || size > kMaxDatagram || key.size() < kTagSize) return false;
    std::uint8_t tag[kTagSize];
    if (!crypto::computeHmacMd5(data, size - kTagSize, reinterpret_cast<const std::uint8_t*>(key.data()), kTagSize, tag)) return false;
    unsigned difference = 0;
    for (std::size_t i = 0; i < kTagSize; ++i) difference |= tag[i] ^ data[size - kTagSize + i];
    return !difference && decode(data, size - kTagSize, out);
}
inline bool sameIntent(const Packet& a, const Packet& b) {
    auto x = a, y = b; x.context.sequence = y.context.sequence = 1;
    std::vector<std::uint8_t> xb, yb;
    return encode(x, xb) && encode(y, yb) && xb == yb;
}
inline bool sameOperation(const Context& a, const Context& b) {
    return a.client == b.client && a.session == b.session && a.command == b.command &&
        a.boot_high == b.boot_high && a.boot_low == b.boot_low;
}
} // namespace robot::platform::protocol_udp
