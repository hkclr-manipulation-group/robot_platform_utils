#ifndef TCP_DATA_CODEC_H
#define TCP_DATA_CODEC_H

#include "tcp_data_payloads.h"
#include "tcp_data_protocol.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace robot::platform::tcp_data {

bool encodeSetWifiRequest(std::uint8_t wifi_enable, const std::string& ssid, const std::string& password,
                          std::vector<std::uint8_t>& out);
bool decodeSetWifiRequest(const std::uint8_t* data, std::size_t size, std::uint8_t& wifi_enable, std::string& ssid,
                          std::string& password);

bool encodeSetEthStaticRequest(const std::string& eth_ip, std::uint8_t eth_prefix, const std::string& eth_netmask,
                               const std::string& eth_gateway, const std::string& eth_dns,
                               std::vector<std::uint8_t>& out);
bool decodeSetEthStaticRequest(const std::uint8_t* data, std::size_t size, std::string& eth_ip, std::uint8_t& eth_prefix,
                               std::string& eth_netmask, std::string& eth_gateway, std::string& eth_dns);

bool encodeNetworkConfigData(const NetworkConfigData& data, std::vector<std::uint8_t>& out);
bool decodeNetworkConfigData(const std::uint8_t* data, std::size_t size, NetworkConfigData& out);

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

bool encodeLogPollRequest(std::uint64_t after_id, std::vector<std::uint8_t>& out);
bool decodeLogPollRequest(const std::uint8_t* data, std::size_t size, std::uint64_t& after_id);

bool encodeLogPollResponse(std::uint64_t latest_id, const std::vector<std::string>& lines,
                           std::vector<std::uint8_t>& out);
bool decodeLogPollResponse(const std::uint8_t* data, std::size_t size, LogPollResult& out);

bool encodeConfigValuePayload(RtConfigType type, const std::vector<std::int32_t>& values,
                              std::vector<std::uint8_t>& out);
bool decodeConfigValuePayload(const std::uint8_t* data, std::size_t size, ConfigValuePayload& out);

bool encodeRpcSessionAuth(const RpcSessionAuth& auth, std::vector<std::uint8_t>& out);
bool decodeRpcSessionAuth(const std::uint8_t* data, std::size_t size, RpcSessionAuth& auth);

bool encodeRpcRequest(const RpcSessionAuth& auth, ServiceId service, std::uint32_t method,
                      const std::vector<std::uint8_t>& body, std::vector<std::uint8_t>& out);
bool decodeRpcRequest(const std::uint8_t* data, std::size_t size, RpcSessionAuth& auth, std::uint32_t& service_id,
                      std::uint32_t& method_id, const std::uint8_t*& body, std::size_t& body_size);

bool encodeBlobBeginPayload(const RpcSessionAuth& auth, const std::string& name, std::uint32_t total_size,
                            std::vector<std::uint8_t>& out);
bool decodeBlobBeginPayload(const std::uint8_t* data, std::size_t size, RpcSessionAuth& auth, std::string& name,
                            std::uint32_t& total_size);

bool encodeRpcResponse(std::uint32_t status_code, const std::vector<std::uint8_t>& body,
                       std::vector<std::uint8_t>& out);
bool decodeRpcResponse(const std::uint8_t* data, std::size_t size, std::uint32_t& status_code,
                       const std::uint8_t*& body, std::size_t& body_size);

/** Wire format: [service_count][service_id][method_count][method_id...]* */
bool encodeRpcCapabilitiesBody(const std::vector<RpcServiceCapability>& services, std::vector<std::uint8_t>& out);
bool decodeRpcCapabilitiesBody(const std::uint8_t* data, std::size_t size,
                               std::vector<RpcServiceCapability>& services);

}  // namespace robot::platform::tcp_data

#endif
