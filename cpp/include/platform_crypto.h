#ifndef PLATFORM_CRYPTO_H
#define PLATFORM_CRYPTO_H

#include <cstdint>
#include <cstddef>

#ifndef HMAC_KEY_SIZE
    #define HMAC_KEY_SIZE           16U
#endif

namespace robot::platform::crypto {
    bool computeHmacMd5(const std::uint8_t* data, std::size_t data_size, const std::uint8_t* key, std::size_t key_size, std::uint8_t* hmac_out);
    bool sendBufferAppendHmac(std::uint8_t* buffer, std::size_t written_size, const std::uint8_t* key, std::size_t key_size, std::uint8_t* hmac_output, std::size_t& updated_buffer_size);
    bool receiveBufferCheckHmac(const std::uint8_t* buffer, std::size_t written_size, const std::uint8_t* key, std::size_t key_size);
}  // namespace robot::platform::crypto

#endif
