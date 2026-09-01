#ifndef PLATFORM_SERIALIZATION_H
#define PLATFORM_SERIALIZATION_H

#include <stdexcept>
#include <cstddef>
#include <cstdint>
#include <variant>
#include <type_traits>
#include <memory>
#include <iostream>

#include "platform_state.h"
#include "platform_crypto.h"

namespace robot::platform::serialization {
    bool getReceivedHeader(const std::uint8_t* buffer, std::size_t buffer_size, uint32_t& magic_header, MessageType& message_type, uint16_t& client_id);
    
    bool toBytes(const SdkCommandReq& value, std::uint8_t* buffer, std::size_t buffer_size, std::size_t& written_size);
    bool toBytes(const SdkCommandRes& value, std::uint8_t* buffer, std::size_t buffer_size, std::size_t& written_size);
    bool toBytes(const SdkConfigReq& value, std::uint8_t* buffer, std::size_t buffer_size, std::size_t& written_size);
    bool toBytes(const SdkConfigRes& value, std::uint8_t* buffer, std::size_t buffer_size, std::size_t& written_size);
    bool toBytes(const SdkHeartbeatReq& value, std::uint8_t* buffer, std::size_t buffer_size, std::size_t& written_size);
    bool toBytes(const SdkSafeguardReq& value, std::uint8_t* buffer, std::size_t buffer_size, std::size_t& written_size);
    bool toBytes(const SdkHandshakeReq& value, std::uint8_t* buffer, std::size_t buffer_size, std::size_t& written_size);
    bool toBytes(const SdkHandshakeRes& value, std::uint8_t* buffer, std::size_t buffer_size, std::size_t& written_size);
    bool toBytes(const SdkReleaseControlReq& value, std::uint8_t* buffer, std::size_t buffer_size, std::size_t& written_size);
    bool toBytes(const SdkReleaseControlRes& value, std::uint8_t* buffer, std::size_t buffer_size, std::size_t& written_size);
    bool toBytes(const SdkRecoveryReq& value, std::uint8_t* buffer, std::size_t buffer_size, std::size_t& written_size);
    bool toBytes(const SdkRecoveryRes& value, std::uint8_t* buffer, std::size_t buffer_size, std::size_t& written_size);
    bool toBytes(const SrvState& value, std::uint8_t* buffer, std::size_t buffer_size, std::size_t& written_size);
    bool toBytes(const CoreRequestVariantPtr& value, std::uint8_t* buffer, std::size_t buffer_size, std::size_t& written_size);
    bool toBytes(const CoreResponseVariantPtr& value, std::uint8_t* buffer, std::size_t buffer_size, std::size_t& written_size);
    bool toBytes(const MonitoringRequestVariantPtr& value, std::uint8_t* buffer, std::size_t buffer_size, std::size_t& written_size);

    bool fromBytes(const std::uint8_t* buffer, std::size_t buffer_size, SdkCommandReq& value);
    bool fromBytes(const std::uint8_t* buffer, std::size_t buffer_size, SdkCommandRes& value);
    bool fromBytes(const std::uint8_t* buffer, std::size_t buffer_size, SdkConfigReq& value);
    bool fromBytes(const std::uint8_t* buffer, std::size_t buffer_size, SdkConfigRes& value);
    bool fromBytes(const std::uint8_t* buffer, std::size_t buffer_size, SdkHeartbeatReq& value);
    bool fromBytes(const std::uint8_t* buffer, std::size_t buffer_size, SdkSafeguardReq& value);
    bool fromBytes(const std::uint8_t* buffer, std::size_t buffer_size, SdkHandshakeReq& value);
    bool fromBytes(const std::uint8_t* buffer, std::size_t buffer_size, SdkHandshakeRes& value);
    bool fromBytes(const std::uint8_t* buffer, std::size_t buffer_size, SdkReleaseControlReq& value);
    bool fromBytes(const std::uint8_t* buffer, std::size_t buffer_size, SdkReleaseControlRes& value);
    bool fromBytes(const std::uint8_t* buffer, std::size_t buffer_size, SdkRecoveryReq& value);
    bool fromBytes(const std::uint8_t* buffer, std::size_t buffer_size, SdkRecoveryRes& value);
    bool fromBytes(const std::uint8_t* buffer, std::size_t buffer_size, SrvState& value);
    bool fromBytes(const std::uint8_t* buffer, std::size_t buffer_size, CoreRequestVariantPtr& value);
    bool fromBytes(const std::uint8_t* buffer, std::size_t buffer_size, CoreResponseVariantPtr& value);
    bool fromBytes(const std::uint8_t* buffer, std::size_t buffer_size, MonitoringRequestVariantPtr& value);

    // Traits to detect if a struct contains a .client_id member field
    template <typename T, typename = std::void_t<>>
    struct has_client_id : std::false_type {};
    template <typename T>
    struct has_client_id<T, std::void_t<decltype(std::declval<T>().client_id)>> : std::true_type {};

    // Traits to detect if a struct contains a .request_client_id member field
    template <typename T, typename = std::void_t<>>
    struct has_request_client_id : std::false_type {};
    template <typename T>
    struct has_request_client_id<T, std::void_t<decltype(std::declval<T>().request_client_id)>> : std::true_type {};

    // Trait to check if a type is ANY std::variant container wrapper
    template <typename T>
    struct is_variant_container : std::false_type {};
    template <typename... Args>
    struct is_variant_container<std::variant<Args...>> : std::true_type {};

    template <typename T>
    struct is_smart_pointer : std::false_type {};
    template <typename T>
    struct is_smart_pointer<std::unique_ptr<T>> : std::true_type {};
    template <typename T>
    struct is_smart_pointer<std::shared_ptr<T>> : std::true_type {};

    template<typename T>
    bool getClientId(const T& value, uint16_t& client_id) {
        using DecayedT = std::decay_t<T>;

        if constexpr (is_smart_pointer<DecayedT>::value) {
            if (!value) {
                return false;
            }
            return getClientId(*value, client_id);
        } else if constexpr (is_variant_container<DecayedT>::value) {
            return std::visit([&client_id](const auto& alternative) -> bool {
                using AltT = std::decay_t<decltype(alternative)>;
                
                if constexpr (has_client_id<AltT>::value) {
                    client_id = alternative.client_id;
                    return true;
                } 
                else if constexpr (has_request_client_id<AltT>::value) {
                    client_id = alternative.request_client_id;
                    return true;
                }
                return false;
            }, value);
        }else {
            if constexpr (has_client_id<DecayedT>::value) {
                client_id = value.client_id;
                return true;
            } 
            else if constexpr (has_request_client_id<DecayedT>::value) {
                client_id = value.request_client_id;
                return true;
            }
        }
        return false;
    }

    template<typename T>
    bool packMessage(const T& value, std::uint8_t* buffer, std::size_t buffer_size, std::size_t& written_size, const std::string* (*get_hmac_key)(uint16_t client_id)){
        written_size = 0;
        if (buffer == nullptr || buffer_size < HMAC_KEY_SIZE) {
            std::cout << "packMessage: buffer too small for HMAC" << std::endl;
            return false;
        }

        std::size_t payload_size = 0;
        uint8_t hmac_output[HMAC_KEY_SIZE];
        uint16_t client_id;
        if (!getClientId(value, client_id)){
            std::cout << "packMessage: getClientId failed" << std::endl;
            return false;
        }

        const std::size_t payload_capacity = buffer_size - HMAC_KEY_SIZE;
        if (!toBytes(value, buffer, payload_capacity, payload_size)
            || payload_size > payload_capacity) {
            std::cout << "packMessage: toBytes failed" << std::endl;
            return false;
        }
        
        //Add security HMAC to the buffer
        const std::string* key = get_hmac_key(client_id);
        if (!key) {
            std::cout << "packMessage: HMAC key not found for client" << std::endl;
            std::cerr << "Failed client_id lookup: " << client_id << std::endl;
            return false;
        }

        const uint8_t* u8_key = reinterpret_cast<const uint8_t*>(key->data());
        if (!crypto::sendBufferAppendHmac(buffer, payload_size, u8_key, HMAC_KEY_SIZE, hmac_output, written_size)){
            std::cout << "packMessage: sendBufferAppendHmac failed" << std::endl;
            return false;
        }
        return true;
    }

    template<typename T>
    bool unpackMessage(const std::uint8_t* buffer, std::size_t written_size, T& value, const std::string* (*get_hmac_key)(uint16_t)){
        uint32_t magic_header;
        uint16_t client_id;
        MessageType message_type;
        const size_t expected_min_size = sizeof(magic_header) + sizeof(message_type) + sizeof(client_id) + HMAC_KEY_SIZE;
        if (written_size < expected_min_size) {
            std::cout << "unpackMessage: written_size < expected_min_size" << std::endl;
            return false;
        }

        getReceivedHeader(buffer, written_size, magic_header, message_type, client_id);
        if (magic_header != MAGIC_HEADER){
            std::cout << "unpackMessage: magic_header != MAGIC_HEADER" << std::endl;
            return false;
        }

        //Check security HMAC in the buffer
        const std::string* key = get_hmac_key(client_id);
        if (!key) {
            std::cout << "unpackMessage: HMAC key not found for client" << std::endl;
            std::cerr << "Failed client_id lookup: " << client_id << std::endl;
            return false;
        }

        const uint8_t* u8_key = reinterpret_cast<const uint8_t*>(key->data());
        if (!crypto::receiveBufferCheckHmac(buffer, written_size, u8_key, HMAC_KEY_SIZE)){
            std::cout << "unpackMessage: receiveBufferCheckHmac failed" << std::endl;
            return false;
        }

        try {
            return fromBytes(buffer, written_size - HMAC_KEY_SIZE, value);
        } catch (const std::exception& e) {
            std::cout << "unpackMessage: invalid or truncated payload: " << e.what() << std::endl;
            return false;
        }
    }
}  // namespace robot::platform::serialization

#endif