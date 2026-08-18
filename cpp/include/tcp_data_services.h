#ifndef TCP_DATA_SERVICES_H
#define TCP_DATA_SERVICES_H

#include <cstdint>
#include <string>
#include <vector>

namespace robot::platform::tcp_data {

enum class ServiceId : std::uint32_t {
    kStorage = 1,
    kTeach = 2,
    kLog = 3,
    kSystem = 4,
};

enum class StorageMethod : std::uint32_t {
    kList = 1,
    kDelete = 2,
    kStat = 3,
    kDownload = 4,
};

enum class SystemMethod : std::uint32_t {
    kGetCapabilities = 1,
    kGetVersion = 2,
};

struct FileEntry {
    std::string name;
    std::uint64_t size = 0;
    std::uint64_t mtime_sec = 0;
};

struct StatResult {
    std::uint64_t size = 0;
    std::uint64_t mtime_sec = 0;
};

bool encodeNameRequest(const std::string& name, std::vector<std::uint8_t>& out);
bool decodeNameRequest(const std::uint8_t* data, std::size_t size, std::string& name);

bool encodePrefixRequest(const std::string& prefix, std::vector<std::uint8_t>& out);
bool decodePrefixRequest(const std::uint8_t* data, std::size_t size, std::string& prefix);

bool encodeFileList(const std::vector<FileEntry>& entries, std::vector<std::uint8_t>& out);
bool decodeFileList(const std::uint8_t* data, std::size_t size, std::vector<FileEntry>& entries);

bool encodeStatResult(const StatResult& stat, std::vector<std::uint8_t>& out);
bool decodeStatResult(const std::uint8_t* data, std::size_t size, StatResult& stat);

bool encodeDownloadMeta(std::uint64_t size, std::vector<std::uint8_t>& out);
bool decodeDownloadMeta(const std::uint8_t* data, std::size_t size, std::uint64_t& size_out);

bool encodeRpcRequest(ServiceId service, std::uint32_t method, const std::vector<std::uint8_t>& body,
                      std::vector<std::uint8_t>& out);
bool decodeRpcRequest(const std::uint8_t* data, std::size_t size, std::uint32_t& service_id,
                      std::uint32_t& method_id, const std::uint8_t*& body, std::size_t& body_size);

bool encodeRpcResponse(std::uint32_t status_code, const std::vector<std::uint8_t>& body,
                         std::vector<std::uint8_t>& out);
bool decodeRpcResponse(const std::uint8_t* data, std::size_t size, std::uint32_t& status_code,
                       const std::uint8_t*& body, std::size_t& body_size);

}  // namespace robot::platform::tcp_data

#endif
