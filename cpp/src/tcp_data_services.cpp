#include "tcp_data_codec.h"

#include "tcp_data_protocol.h"

#include <cstring>

namespace robot::platform::tcp_data {
namespace {

bool readU16(const std::uint8_t* data, std::size_t size, std::size_t& offset, std::uint16_t& out) {
    if (offset + sizeof(std::uint16_t) > size) {
        return false;
    }
    std::memcpy(&out, data + offset, sizeof(out));
    offset += sizeof(std::uint16_t);
    return true;
}

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

bool writeU16(std::vector<std::uint8_t>& out, std::uint16_t value) {
    const auto* bytes = reinterpret_cast<const std::uint8_t*>(&value);
    out.insert(out.end(), bytes, bytes + sizeof(value));
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

bool writeI32(std::vector<std::uint8_t>& out, std::int32_t value) {
    const auto* bytes = reinterpret_cast<const std::uint8_t*>(&value);
    out.insert(out.end(), bytes, bytes + sizeof(value));
    return true;
}

bool readI32(const std::uint8_t* data, std::size_t size, std::size_t& offset, std::int32_t& out) {
    if (offset + sizeof(std::int32_t) > size) {
        return false;
    }
    std::memcpy(&out, data + offset, sizeof(out));
    offset += sizeof(std::int32_t);
    return true;
}

bool writeU8(std::vector<std::uint8_t>& out, std::uint8_t value) {
    out.push_back(value);
    return true;
}

bool readU8(const std::uint8_t* data, std::size_t size, std::size_t& offset, std::uint8_t& out) {
    if (offset + 1 > size) {
        return false;
    }
    out = data[offset++];
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

bool encodeLogPollRequest(std::uint64_t after_id, std::vector<std::uint8_t>& out) {
    out.clear();
    writeU64(out, after_id);
    return true;
}

bool decodeLogPollRequest(const std::uint8_t* data, std::size_t size, std::uint64_t& after_id) {
    std::size_t offset = 0;
    return readU64(data, size, offset, after_id);
}

bool encodeLogPollResponse(std::uint64_t latest_id, const std::vector<std::string>& lines,
                           std::vector<std::uint8_t>& out) {
    out.clear();
    writeU64(out, latest_id);
    writeU32(out, static_cast<std::uint32_t>(lines.size()));
    for (const auto& line : lines) {
        if (!encodeStringField(line, out)) {
            out.clear();
            return false;
        }
    }
    return true;
}

bool decodeLogPollResponse(const std::uint8_t* data, std::size_t size, LogPollResult& out) {
    std::size_t offset = 0;
    std::uint32_t count = 0;
    if (!readU64(data, size, offset, out.latest_id) || !readU32(data, size, offset, count)) {
        return false;
    }
    out.lines.clear();
    out.lines.reserve(count);
    for (std::uint32_t i = 0; i < count; ++i) {
        std::string line;
        if (!decodeStringField(data, size, offset, line)) {
            return false;
        }
        out.lines.push_back(std::move(line));
    }
    return offset == size;
}

bool encodeConfigValuePayload(RtConfigType type, const std::vector<std::int32_t>& values,
                              std::vector<std::uint8_t>& out) {
    out.clear();
    writeU32(out, static_cast<std::uint32_t>(type));
    writeU32(out, static_cast<std::uint32_t>(values.size()));
    for (const std::int32_t value : values) {
        writeI32(out, value);
    }
    return true;
}

bool decodeConfigValuePayload(const std::uint8_t* data, std::size_t size, ConfigValuePayload& out) {
    std::size_t offset = 0;
    std::uint32_t type = 0;
    std::uint32_t count = 0;
    if (!readU32(data, size, offset, type) || !readU32(data, size, offset, count)) {
        return false;
    }
    out.type = static_cast<RtConfigType>(type);
    out.values.clear();
    out.values.reserve(count);
    for (std::uint32_t i = 0; i < count; ++i) {
        std::int32_t value = 0;
        if (!readI32(data, size, offset, value)) {
            return false;
        }
        out.values.push_back(value);
    }
    return offset == size;
}

bool encodeSetWifiRequest(std::uint8_t wifi_enable, const std::string& ssid, const std::string& password,
                          std::vector<std::uint8_t>& out) {
    out.clear();
    out.push_back(wifi_enable);
    if (!encodeStringField(ssid, out) || !encodeStringField(password, out)) {
        out.clear();
        return false;
    }
    return true;
}

bool decodeSetWifiRequest(const std::uint8_t* data, std::size_t size, std::uint8_t& wifi_enable, std::string& ssid,
                          std::string& password) {
    if (size < 1) {
        return false;
    }
    wifi_enable = data[0];
    std::size_t offset = 1;
    return decodeStringField(data, size, offset, ssid) && decodeStringField(data, size, offset, password);
}

bool encodeSetEthStaticRequest(const std::string& eth_ip, std::uint8_t eth_prefix, const std::string& eth_netmask,
                               const std::string& eth_gateway, const std::string& eth_dns,
                               std::vector<std::uint8_t>& out) {
    out.clear();
    if (!encodeStringField(eth_ip, out) || !writeU8(out, eth_prefix) || !encodeStringField(eth_netmask, out)
        || !encodeStringField(eth_gateway, out) || !encodeStringField(eth_dns, out)) {
        out.clear();
        return false;
    }
    return true;
}

bool decodeSetEthStaticRequest(const std::uint8_t* data, std::size_t size, std::string& eth_ip, std::uint8_t& eth_prefix,
                               std::string& eth_netmask, std::string& eth_gateway, std::string& eth_dns) {
    std::size_t offset = 0;
    if (!decodeStringField(data, size, offset, eth_ip) || !readU8(data, size, offset, eth_prefix)
        || !decodeStringField(data, size, offset, eth_netmask) || !decodeStringField(data, size, offset, eth_gateway)
        || !decodeStringField(data, size, offset, eth_dns)) {
        return false;
    }
    // Legacy clients may append an unused server_ip field (same as eth_ip).
    std::string legacy_server_ip;
    decodeStringField(data, size, offset, legacy_server_ip);
    return true;
}

bool encodeNetworkConfigData(const NetworkConfigData& data, std::vector<std::uint8_t>& out) {
    out.clear();
    writeU32(out, data.command_status);
    writeU8(out, data.action);
    writeU8(out, data.wifi_enable);
    if (!encodeStringField(data.wifi_ssid, out) || !encodeStringField(data.wifi_password, out)
        || !encodeStringField(data.eth_ip, out) || !writeU8(out, data.eth_prefix)
        || !encodeStringField(data.eth_netmask, out) || !encodeStringField(data.eth_gateway, out)
        || !encodeStringField(data.eth_dns, out)) {
        out.clear();
        return false;
    }
    return true;
}

bool decodeNetworkConfigData(const std::uint8_t* data, std::size_t size, NetworkConfigData& out) {
    std::size_t offset = 0;
    std::uint32_t command_status = 0;
    if (!readU32(data, size, offset, command_status) || !readU8(data, size, offset, out.action)
        || !readU8(data, size, offset, out.wifi_enable) || !decodeStringField(data, size, offset, out.wifi_ssid)
        || !decodeStringField(data, size, offset, out.wifi_password) || !decodeStringField(data, size, offset, out.eth_ip)
        || !readU8(data, size, offset, out.eth_prefix) || !decodeStringField(data, size, offset, out.eth_netmask)
        || !decodeStringField(data, size, offset, out.eth_gateway) || !decodeStringField(data, size, offset, out.eth_dns)) {
        return false;
    }
    out.command_status = command_status;
    // Legacy servers may append an unused server_ip field (same as eth_ip).
    std::string legacy_server_ip;
    decodeStringField(data, size, offset, legacy_server_ip);
    return true;
}

bool encodeRpcSessionAuth(const RpcSessionAuth& auth, std::vector<std::uint8_t>& out) {
    out.clear();
    writeU16(out, auth.client_id);
    writeU32(out, auth.session_id);
    writeU32(out, auth.sequence_id);
    return out.size() == kRpcSessionAuthSize;
}

bool decodeRpcSessionAuth(const std::uint8_t* data, std::size_t size, RpcSessionAuth& auth) {
    if (size < kRpcSessionAuthSize) {
        return false;
    }
    std::size_t offset = 0;
    return readU16(data, size, offset, auth.client_id)
        && readU32(data, size, offset, auth.session_id)
        && readU32(data, size, offset, auth.sequence_id);
}

bool encodeRpcRequest(const RpcSessionAuth& auth, ServiceId service, std::uint32_t method,
                      const std::vector<std::uint8_t>& body, std::vector<std::uint8_t>& out) {
    out.clear();
    if (!encodeRpcSessionAuth(auth, out)) {
        return false;
    }
    writeU32(out, static_cast<std::uint32_t>(service));
    writeU32(out, method);
    out.insert(out.end(), body.begin(), body.end());
    return true;
}

bool decodeRpcRequest(const std::uint8_t* data, std::size_t size, RpcSessionAuth& auth, std::uint32_t& service_id,
                      std::uint32_t& method_id, const std::uint8_t*& body, std::size_t& body_size) {
    if (size < kRpcSessionAuthSize + sizeof(RpcRequestHeader)) {
        return false;
    }
    std::size_t offset = 0;
    if (!readU16(data, size, offset, auth.client_id)
        || !readU32(data, size, offset, auth.session_id)
        || !readU32(data, size, offset, auth.sequence_id)
        || !readU32(data, size, offset, service_id)
        || !readU32(data, size, offset, method_id)) {
        return false;
    }
    body = data + offset;
    body_size = size - offset;
    return true;
}

bool encodeBlobBeginPayload(const RpcSessionAuth& auth, const std::string& name, std::uint32_t total_size,
                            std::vector<std::uint8_t>& out) {
    out.clear();
    if (!encodeRpcSessionAuth(auth, out)) {
        return false;
    }
    if (!encodeStringField(name, out)) {
        return false;
    }
    writeU32(out, total_size);
    return true;
}

bool decodeBlobBeginPayload(const std::uint8_t* data, std::size_t size, RpcSessionAuth& auth, std::string& name,
                            std::uint32_t& total_size) {
    if (size < kRpcSessionAuthSize) {
        return false;
    }
    std::size_t offset = 0;
    if (!readU16(data, size, offset, auth.client_id)
        || !readU32(data, size, offset, auth.session_id)
        || !readU32(data, size, offset, auth.sequence_id)
        || !decodeStringField(data, size, offset, name)) {
        return false;
    }
    return readU32(data, size, offset, total_size);
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

bool encodeRpcCapabilitiesBody(const std::vector<RpcServiceCapability>& services, std::vector<std::uint8_t>& out) {
    out.clear();
    writeU32(out, static_cast<std::uint32_t>(services.size()));
    for (const RpcServiceCapability& service : services) {
        writeU32(out, service.service_id);
        writeU32(out, static_cast<std::uint32_t>(service.method_ids.size()));
        for (std::uint32_t method_id : service.method_ids) {
            writeU32(out, method_id);
        }
    }
    return true;
}

bool decodeRpcCapabilitiesBody(const std::uint8_t* data, std::size_t size,
                               std::vector<RpcServiceCapability>& services) {
    services.clear();
    std::size_t offset = 0;
    std::uint32_t service_count = 0;
    if (!readU32(data, size, offset, service_count)) {
        return false;
    }
    services.reserve(service_count);
    for (std::uint32_t service_i = 0; service_i < service_count; ++service_i) {
        RpcServiceCapability entry;
        std::uint32_t method_count = 0;
        if (!readU32(data, size, offset, entry.service_id)
            || !readU32(data, size, offset, method_count)) {
            return false;
        }
        entry.method_ids.reserve(method_count);
        for (std::uint32_t method_i = 0; method_i < method_count; ++method_i) {
            std::uint32_t method_id = 0;
            if (!readU32(data, size, offset, method_id)) {
                return false;
            }
            entry.method_ids.push_back(method_id);
        }
        services.push_back(std::move(entry));
    }
    return offset == size;
}

}  // namespace robot::platform::tcp_data
