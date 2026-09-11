#include <cstdint>
#include <iostream>
#include <vector>

#include "tcp_data_codec.h"
#include "tcp_data_service_ids.h"

namespace {

using robot::platform::tcp_data::ConfigMethod;
using robot::platform::tcp_data::LogMethod;
using robot::platform::tcp_data::NetworkMethod;
using robot::platform::tcp_data::RpcServiceCapability;
using robot::platform::tcp_data::RuntimeSettingsMethod;
using robot::platform::tcp_data::ServiceId;
using robot::platform::tcp_data::StorageMethod;
using robot::platform::tcp_data::SystemMethod;

std::vector<RpcServiceCapability> buildRtControlCapabilities() {
    return {
        {static_cast<std::uint32_t>(ServiceId::kStorage),
         {static_cast<std::uint32_t>(StorageMethod::kList),
          static_cast<std::uint32_t>(StorageMethod::kDelete),
          static_cast<std::uint32_t>(StorageMethod::kStat),
          static_cast<std::uint32_t>(StorageMethod::kDownload)}},
        {static_cast<std::uint32_t>(ServiceId::kSystem),
         {static_cast<std::uint32_t>(SystemMethod::kGetCapabilities),
          static_cast<std::uint32_t>(SystemMethod::kGetVersion),
          static_cast<std::uint32_t>(SystemMethod::kResetEncoderLastPosition),
          static_cast<std::uint32_t>(SystemMethod::kGetArmSerialNumber),
          static_cast<std::uint32_t>(SystemMethod::kSetArmSerialNumber),
          static_cast<std::uint32_t>(SystemMethod::kFlushRuntimeConfig)}},
        {static_cast<std::uint32_t>(ServiceId::kNetwork),
         {static_cast<std::uint32_t>(NetworkMethod::kGet),
          static_cast<std::uint32_t>(NetworkMethod::kSetWifi),
          static_cast<std::uint32_t>(NetworkMethod::kSetEthStatic),
          static_cast<std::uint32_t>(NetworkMethod::kFactoryReset)}},
        {static_cast<std::uint32_t>(ServiceId::kLog),
         {static_cast<std::uint32_t>(LogMethod::kPoll)}},
        {static_cast<std::uint32_t>(ServiceId::kConfig),
         {static_cast<std::uint32_t>(ConfigMethod::kGet),
          static_cast<std::uint32_t>(ConfigMethod::kSet)}},
        {static_cast<std::uint32_t>(ServiceId::kRuntimeSettings),
         {static_cast<std::uint32_t>(RuntimeSettingsMethod::kFlush),
          static_cast<std::uint32_t>(RuntimeSettingsMethod::kListProfiles),
          static_cast<std::uint32_t>(RuntimeSettingsMethod::kActivateProfile),
          static_cast<std::uint32_t>(RuntimeSettingsMethod::kCreateProfile),
          static_cast<std::uint32_t>(RuntimeSettingsMethod::kGetRuntimeConfigJson),
          static_cast<std::uint32_t>(RuntimeSettingsMethod::kGetArmEffectiveJson),
          static_cast<std::uint32_t>(RuntimeSettingsMethod::kSetArmSafetyJson)}},
    };
}

}  // namespace

int main() {
    const auto expected = buildRtControlCapabilities();
    std::vector<std::uint8_t> encoded;
    if (!robot::platform::tcp_data::encodeRpcCapabilitiesBody(expected, encoded)) {
        std::cerr << "encodeRpcCapabilitiesBody failed\n";
        return 1;
    }

    std::vector<RpcServiceCapability> decoded;
    if (!robot::platform::tcp_data::decodeRpcCapabilitiesBody(encoded.data(), encoded.size(), decoded)) {
        std::cerr << "decodeRpcCapabilitiesBody failed\n";
        return 1;
    }

    if (decoded.size() != expected.size()) {
        std::cerr << "service count mismatch: " << decoded.size() << " vs " << expected.size() << '\n';
        return 1;
    }

    std::size_t total_methods = 0;
    for (std::size_t i = 0; i < expected.size(); ++i) {
        if (decoded[i].service_id != expected[i].service_id
            || decoded[i].method_ids != expected[i].method_ids) {
            std::cerr << "capability mismatch at service index " << i << '\n';
            return 1;
        }
        total_methods += decoded[i].method_ids.size();
    }

    if (total_methods != 24) {
        std::cerr << "expected 24 registered methods, got " << total_methods << '\n';
        return 1;
    }

    std::cout << "rpc capabilities codec ok (" << expected.size() << " services, " << total_methods << " methods)\n";
    return 0;
}
