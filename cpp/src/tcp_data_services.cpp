#include "tcp_data_services.h"

#include "tcp_data_protocol.h"

#include <cstring>

namespace robot::platform::tcp_data {
namespace {

bool readU32(const std::uint8_t* data, std::size_t size, std::size_t& offset, std::uint32_t& out) {
    if (offset + sizeof(std::uint32_t) > size) {
        return false;
    }
    std::memcpy(&out, data + offset, sizeof(out));
    offset += sizeof(std::uint32_t);
    return true;
}

bool readU64(const std::uint8_t* data, std::size_t size, std::size_t& offset, std::uint64_t& out) {
    if (offset + sizeof(std::uint64_t) > size) {
        return false;
    }
    std::memcpy(&out, data + offset, sizeof(out));
    offset += sizeof(std::uint64_t);
    return true;
}

bool writeU32(std::vector<std::uint8_t>& out, std::uint32_t value) {
    const auto* bytes = reinterpret_cast<const std::uint8_t*>(&value);
    out.insert(out.end(), bytes, bytes + sizeof(value));
    return true;
}

bool writeU64(std::vector<std::uint8_t>& out, std::uint64_t value) {
    const auto* bytes = reinterpret_cast<const std::uint8_t*>(&value);
    out.insert(out.end(), bytes, bytes + sizeof(value));
    return true;
}

bool decodeStringField(const std::uint8_t* data, std::size_t size, std::size_t& offset, std::string& out) {
    std::uint32_t len = 0;
    if (!readU32(data, size, offset, len)) {
        return false;
    }
    if (offset + len > size) {
        return false;
    }
    out.assign(reinterpret_cast<const char*>(data + offset), len);
    offset += len;
    return true;
}

bool encodeStringField(const std::string& value, std::vector<std::uint8_t>& out) {
    if (value.size() > 0xFFFFFFFFu) {
        return false;
    }
    writeU32(out, static_cast<std::uint32_t>(value.size()));
    out.insert(out.end(), value.begin(), value.end());
    return true;
}

}  // namespace

bool encodeNameRequest(const std::string& name, std::vector<std::uint8_t>& out) {
    out.clear();
    return encodeStringField(name, out);
}

bool decodeNameRequest(const std::uint8_t* data, std::size_t size, std::string& name) {
    std::size_t offset = 0;
    return decodeStringField(data, size, offset, name);
}

bool encodePrefixRequest(const std::string& prefix, std::vector<std::uint8_t>& out) {
    return encodeNameRequest(prefix, out);
}

bool decodePrefixRequest(const std::uint8_t* data, std::size_t size, std::string& prefix) {
    return decodeNameRequest(data, size, prefix);
}

bool encodeFileList(const std::vector<FileEntry>& entries, std::vector<std::uint8_t>& out) {
    out.clear();
    writeU32(out, static_cast<std::uint32_t>(entries.size()));
    for (const auto& entry : entries) {
        if (!encodeStringField(entry.name, out)) {
            return false;
        }
        writeU64(out, entry.size);
        writeU64(out, entry.mtime_sec);
    }
    return true;
}

bool decodeFileList(const std::uint8_t* data, std::size_t size, std::vector<FileEntry>& entries) {
    entries.clear();
    std::size_t offset = 0;
    std::uint32_t count = 0;
    if (!readU32(data, size, offset, count)) {
        return false;
    }
    entries.reserve(count);
    for (std::uint32_t i = 0; i < count; ++i) {
        FileEntry entry;
        if (!decodeStringField(data, size, offset, entry.name)) {
            return false;
        }
        if (!readU64(data, size, offset, entry.size) || !readU64(data, size, offset, entry.mtime_sec)) {
            return false;
        }
        entries.push_back(std::move(entry));
    }
    return true;
}

bool encodeStatResult(const StatResult& stat, std::vector<std::uint8_t>& out) {
    out.clear();
    writeU64(out, stat.size);
    writeU64(out, stat.mtime_sec);
    return true;
}

bool decodeStatResult(const std::uint8_t* data, std::size_t size, StatResult& stat) {
    std::size_t offset = 0;
    return readU64(data, size, offset, stat.size) && readU64(data, size, offset, stat.mtime_sec);
}

bool encodeDownloadMeta(std::uint64_t file_size, std::vector<std::uint8_t>& out) {
    out.clear();
    writeU64(out, file_size);
    return true;
}

bool decodeDownloadMeta(const std::uint8_t* data, std::size_t size, std::uint64_t& size_out) {
    std::size_t offset = 0;
    return readU64(data, size, offset, size_out);
}

bool encodeRpcRequest(ServiceId service, std::uint32_t method, const std::vector<std::uint8_t>& body,
                      std::vector<std::uint8_t>& out) {
    out.clear();
    writeU32(out, static_cast<std::uint32_t>(service));
    writeU32(out, method);
    out.insert(out.end(), body.begin(), body.end());
    return true;
}

bool decodeRpcRequest(const std::uint8_t* data, std::size_t size, std::uint32_t& service_id,
                      std::uint32_t& method_id, const std::uint8_t*& body, std::size_t& body_size) {
    if (size < kRpcHeaderSize) {
        return false;
    }
    std::size_t offset = 0;
    if (!readU32(data, size, offset, service_id) || !readU32(data, size, offset, method_id)) {
        return false;
    }
    body = data + offset;
    body_size = size - offset;
    return true;
}

bool encodeRpcResponse(std::uint32_t status_code, const std::vector<std::uint8_t>& body,
                         std::vector<std::uint8_t>& out) {
    out.clear();
    writeU32(out, status_code);
    out.insert(out.end(), body.begin(), body.end());
    return true;
}

bool decodeRpcResponse(const std::uint8_t* data, std::size_t size, std::uint32_t& status_code,
                       const std::uint8_t*& body, std::size_t& body_size) {
    if (size < sizeof(RpcResponseHeader)) {
        return false;
    }
    std::size_t offset = 0;
    if (!readU32(data, size, offset, status_code)) {
        return false;
    }
    body = data + offset;
    body_size = size - offset;
    return true;
}

}  // namespace robot::platform::tcp_data
