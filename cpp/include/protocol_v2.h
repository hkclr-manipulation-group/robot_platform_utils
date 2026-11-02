#pragma once

// Canonical wire contract. Keep the SDK and RT copies byte-identical.
// No native struct layout, host byte order, sockets or external dependencies.
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace robot::platform::protocol_v2 {
constexpr std::uint32_t kServiceId = 9, kDescribeMethod = 1;
constexpr std::uint32_t kMagic = 0x32505253; // bytes "SRP2"
constexpr std::uint16_t kMajor = 2, kMinor = 0, kCritical = 1;
constexpr std::size_t kHeaderSize = 32, kMaxMessage = 16384;
constexpr std::size_t kMaxComponents = 64, kMaxJoints = 256, kMaxName = 64, kMaxFields = 256;
constexpr std::uint64_t kTopology = 1, kOptionalFields = 2;
constexpr std::uint64_t kImplementedFeatures = kTopology | kOptionalFields;
enum class Kind : std::uint16_t { Hello = 1, Profile = 2 };
enum class ComponentKind : std::uint16_t { Arm = 1, Gripper = 2, Head = 3, Waist = 4, Base = 5 };
enum class Error { Ok, Malformed, UnsupportedVersion, UnsupportedRequiredField, MissingCapability };
struct Hello {
    std::uint64_t request_id = 1, required_features = 0;
    std::uint16_t min_major = 2, max_major = 2, max_minor = 0;
};
struct Component {
    std::uint32_t id = 0;
    std::uint16_t kind = 0, joint_count = 0;
    std::string name;
    // Only V2 command types implemented by this component may be advertised.
    std::uint64_t command_features = 0;
};
struct Profile {
    std::uint64_t request_id = 1, features = kImplementedFeatures;
    std::uint64_t boot_high = 0, boot_low = 0, topology_revision = 1;
    std::uint64_t configuration_revision = 0;
    std::uint16_t major = 2, minor = 0, motion_major = 1;
    std::uint32_t max_message = kMaxMessage;
    std::uint16_t max_components = kMaxComponents, max_joints = kMaxJoints;
    std::vector<Component> components;
};

class Reader {
public:
    Reader(const std::uint8_t* data, std::size_t size) : data_(data), size_(size) {}
    std::size_t remaining() const { return size_ - offset_; }
    bool integer(std::uint64_t& value, unsigned bytes) {
        if (!data_ || bytes > 8 || bytes > remaining()) return false;
        value = 0;
        for (unsigned i = 0; i < bytes; ++i) value |= std::uint64_t(data_[offset_++]) << (8 * i);
        return true;
    }
    template<class T> bool get(T& value) {
        static_assert(std::is_unsigned<T>::value, "unsigned wire integer required");
        std::uint64_t temporary;
        if (!integer(temporary, sizeof(T))) return false;
        value = static_cast<T>(temporary);
        return true;
    }
    bool take(std::size_t count, const std::uint8_t*& data) {
        if (!data_ || count > remaining()) return false;
        data = data_ + offset_;
        offset_ += count;
        return true;
    }
private:
    const std::uint8_t* data_;
    std::size_t size_, offset_ = 0;
};

class Writer {
public:
    template<class T> bool put(T value) {
        static_assert(std::is_unsigned<T>::value, "unsigned wire integer required");
        if (sizeof(T) > kMaxMessage - data.size()) return false;
        for (unsigned i = 0; i < sizeof(T); ++i) data.push_back(static_cast<std::uint8_t>(value >> (8 * i)));
        return true;
    }
    bool append(const std::uint8_t* bytes, std::size_t size) {
        if (size > kMaxMessage - data.size() || (size && !bytes)) return false;
        if (size) data.insert(data.end(), bytes, bytes + size);
        return true;
    }
    bool field(std::uint16_t tag, const Writer& body, std::uint16_t flags = kCritical) {
        if (!tag || body.data.size() + 8 > kMaxMessage - data.size()) return false;
        return put(tag) && put(flags) && put(static_cast<std::uint32_t>(body.data.size())) &&
            append(body.data.data(), body.data.size());
    }
    template<class T> bool scalar(std::uint16_t tag, T value, std::uint16_t flags = kCritical) {
        Writer body;
        return body.put(value) && field(tag, body, flags);
    }
    std::vector<std::uint8_t> data;
};

struct Field { std::uint16_t tag = 0, flags = 0; const std::uint8_t* data = nullptr; std::uint32_t size = 0; };
inline bool nextField(Reader& in, Field& field) {
    return in.get(field.tag) && field.tag && in.get(field.flags) && !(field.flags & ~kCritical) &&
        in.get(field.size) && in.take(field.size, field.data);
}
template<class T> inline bool scalar(const Field& field, T& value) {
    Reader in(field.data, field.size);
    return field.size == sizeof(T) && in.get(value);
}
inline bool validName(const std::string& name) {
    if (name.empty() || name.size() > kMaxName) return false;
    for (unsigned char c : name) if (c < 0x21 || c > 0x7e) return false;
    return true;
}
inline Error open(const std::uint8_t* data, std::size_t size, Kind kind,
                  std::uint64_t& request, Reader& body) {
    if (!data || size < kHeaderSize || size > kMaxMessage) return Error::Malformed;
    Reader in(data, size);
    std::uint32_t magic, header, payload, reserved;
    std::uint16_t major, minor, type, flags;
    if (!in.get(magic) || !in.get(major) || !in.get(minor) || !in.get(type) || !in.get(flags) ||
        !in.get(header) || !in.get(payload) || !in.get(request) || !in.get(reserved) ||
        magic != kMagic || type != static_cast<std::uint16_t>(kind) || flags || reserved ||
        header != kHeaderSize || !request || payload != size - kHeaderSize) return Error::Malformed;
    if (major != kMajor) return Error::UnsupportedVersion;
    // Future minors can add optional fields; required unknown fields still reject.
    body = Reader(data + kHeaderSize, payload);
    return Error::Ok;
}
inline bool seal(Kind kind, std::uint64_t request, const Writer& body, std::vector<std::uint8_t>& out) {
    if (!request || body.data.size() > kMaxMessage - kHeaderSize) return false;
    Writer header;
    if (!header.put(kMagic) || !header.put(kMajor) || !header.put(kMinor) ||
        !header.put(static_cast<std::uint16_t>(kind)) || !header.put(std::uint16_t(0)) ||
        !header.put(std::uint32_t(kHeaderSize)) || !header.put(std::uint32_t(body.data.size())) ||
        !header.put(request) || !header.put(std::uint32_t(0)) ||
        !header.append(body.data.data(), body.data.size())) return false;
    out = std::move(header.data);
    return true;
}
inline bool encode(const Hello& value, std::vector<std::uint8_t>& out) {
    if (!value.min_major || value.min_major > value.max_major) return false;
    Writer versions, body;
    return versions.put(value.min_major) && versions.put(value.max_major) && versions.put(value.max_minor) &&
        versions.put(std::uint16_t(0)) && body.field(1, versions) && body.scalar(2, value.required_features) &&
        seal(Kind::Hello, value.request_id, body, out);
}
inline Error decode(const std::uint8_t* data, std::size_t size, Hello& out) {
    Hello value; Reader body(nullptr, 0);
    auto error = open(data, size, Kind::Hello, value.request_id, body);
    if (error != Error::Ok) return error;
    unsigned seen = 0, count = 0;
    while (body.remaining()) {
        Field field;
        if (++count > kMaxFields || !nextField(body, field)) return Error::Malformed;
        if (field.tag == 1) {
            std::uint16_t reserved;
            Reader versions(field.data, field.size);
            if ((seen & 1) || field.size != 8 || !versions.get(value.min_major) ||
                !versions.get(value.max_major) || !versions.get(value.max_minor) || !versions.get(reserved) ||
                reserved || !value.min_major || value.min_major > value.max_major) return Error::Malformed;
            seen |= 1;
        } else if (field.tag == 2) {
            if ((seen & 2) || !scalar(field, value.required_features)) return Error::Malformed;
            seen |= 2;
        } else if (field.flags & kCritical) return Error::UnsupportedRequiredField;
    }
    if (seen != 3) return Error::Malformed;
    out = value; return Error::Ok;
}
inline bool encodeComponent(const Component& value, Writer& body) {
    if (!value.id || !value.kind || value.joint_count > kMaxJoints || !validName(value.name)) return false;
    Writer name;
    return name.append(reinterpret_cast<const std::uint8_t*>(value.name.data()), value.name.size()) &&
        body.scalar(1, value.id) && body.scalar(2, value.kind) && body.scalar(3, value.joint_count) &&
        body.field(4, name) && body.scalar(5, value.command_features);
}
inline Error decodeComponent(const Field& record, Component& out) {
    Reader body(record.data, record.size); Component value;
    unsigned seen = 0, count = 0;
    while (body.remaining()) {
        Field field;
        if (++count > kMaxFields || !nextField(body, field)) return Error::Malformed;
        if (field.tag >= 1 && field.tag <= 5) {
            const auto bit = 1u << (field.tag - 1);
            if (seen & bit) return Error::Malformed;
            seen |= bit;
            switch (field.tag) {
                case 1: if (!scalar(field, value.id) || !value.id) return Error::Malformed; break;
                case 2: if (!scalar(field, value.kind) || !value.kind) return Error::Malformed; break;
                case 3: if (!scalar(field, value.joint_count) || value.joint_count > kMaxJoints) return Error::Malformed; break;
                case 4:
                    if (!field.size || field.size > kMaxName) return Error::Malformed;
                    value.name.assign(reinterpret_cast<const char*>(field.data), field.size);
                    if (!validName(value.name)) return Error::Malformed;
                    break;
                case 5: if (!scalar(field, value.command_features)) return Error::Malformed; break;
            }
        } else if (field.flags & kCritical) return Error::UnsupportedRequiredField;
    }
    if (seen != 31) return Error::Malformed;
    out = std::move(value); return Error::Ok;
}
inline bool valid(const Profile& value) {
    if (!value.request_id || value.major != kMajor || !value.motion_major ||
        !(value.boot_high || value.boot_low) || !value.topology_revision ||
        value.max_message < kHeaderSize || value.max_message > kMaxMessage ||
        !value.max_components || value.max_components > kMaxComponents ||
        !value.max_joints || value.max_joints > kMaxJoints || value.components.size() > value.max_components) return false;
    for (std::size_t i = 0; i < value.components.size(); ++i) {
        const auto& c = value.components[i];
        if (!c.id || !c.kind || !validName(c.name) || c.joint_count > value.max_joints) return false;
        for (std::size_t j = 0; j < i; ++j)
            if (c.id == value.components[j].id || c.name == value.components[j].name) return false;
    }
    return true;
}
inline bool encode(const Profile& value, std::vector<std::uint8_t>& out) {
    if (!valid(value)) return false;
    Writer body, versions, limits, boot;
    if (!versions.put(value.major) || !versions.put(value.minor) || !versions.put(value.motion_major) ||
        !versions.put(std::uint16_t(0)) || !body.field(10, versions) || !body.scalar(11, value.features) ||
        !limits.put(value.max_message) || !limits.put(value.max_components) || !limits.put(value.max_joints) ||
        !body.field(12, limits) || !boot.put(value.boot_high) || !boot.put(value.boot_low) ||
        !body.field(13, boot) || !body.scalar(14, value.topology_revision)) return false;
    if (value.configuration_revision && !body.scalar(15, value.configuration_revision, 0)) return false;
    for (const auto& c : value.components) {
        Writer record;
        if (!encodeComponent(c, record) || !body.field(20, record)) return false;
    }
    return seal(Kind::Profile, value.request_id, body, out);
}
inline Error decode(const std::uint8_t* data, std::size_t size, Profile& out) {
    Profile value; Reader body(nullptr, 0);
    auto error = open(data, size, Kind::Profile, value.request_id, body);
    if (error != Error::Ok) return error;
    unsigned seen = 0, count = 0;
    bool configuration_seen = false;
    while (body.remaining()) {
        Field field;
        if (++count > kMaxFields || !nextField(body, field)) return Error::Malformed;
        if (field.tag >= 10 && field.tag <= 14) {
            const auto bit = 1u << (field.tag - 10);
            if (seen & bit) return Error::Malformed;
            seen |= bit;
            Reader record(field.data, field.size);
            std::uint16_t reserved;
            switch (field.tag) {
                case 10:
                    if (field.size != 8 || !record.get(value.major) || !record.get(value.minor) ||
                        !record.get(value.motion_major) || !record.get(reserved) || reserved) return Error::Malformed;
                    break;
                case 11: if (!scalar(field, value.features)) return Error::Malformed; break;
                case 12:
                    if (field.size != 8 || !record.get(value.max_message) || !record.get(value.max_components) ||
                        !record.get(value.max_joints)) return Error::Malformed;
                    break;
                case 13:
                    if (field.size != 16 || !record.get(value.boot_high) || !record.get(value.boot_low)) return Error::Malformed;
                    break;
                case 14: if (!scalar(field, value.topology_revision)) return Error::Malformed; break;
            }
        } else if (field.tag == 15) {
            if (configuration_seen || !scalar(field, value.configuration_revision)) return Error::Malformed;
            configuration_seen = true;
        } else if (field.tag == 20) {
            if (value.components.size() >= kMaxComponents) return Error::Malformed;
            Component c;
            error = decodeComponent(field, c);
            if (error != Error::Ok) return error;
            value.components.push_back(std::move(c));
        } else if (field.flags & kCritical) return Error::UnsupportedRequiredField;
    }
    if (seen != 31 || !valid(value)) return Error::Malformed;
    out = std::move(value); return Error::Ok;
}
inline Error negotiate(const Hello& hello, Profile& profile) {
    if (hello.min_major > kMajor || hello.max_major < kMajor) return Error::UnsupportedVersion;
    if ((hello.required_features & profile.features) != hello.required_features) return Error::MissingCapability;
    profile.request_id = hello.request_id;
    profile.major = kMajor; profile.minor = (std::min)(hello.max_minor, kMinor);
    return valid(profile) ? Error::Ok : Error::Malformed;
}
// A response is usable only for this request and all requested requirements.
inline bool matches(const Hello& hello, const Profile& profile) {
    return valid(profile) && profile.request_id == hello.request_id && profile.major >= hello.min_major &&
        profile.major <= hello.max_major && profile.minor <= hello.max_minor &&
        (profile.features & hello.required_features) == hello.required_features;
}
} // namespace robot::platform::protocol_v2
