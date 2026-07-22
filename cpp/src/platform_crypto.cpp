#include "platform_crypto.h"

#include <openssl/evp.h>
#include <openssl/hmac.h>

#include <cstdint>
#include <cstddef>
#include <cstring>

namespace robot::platform::crypto {
    constexpr std::size_t MD5_HASH_SIZE = 16;

    bool computeHmacMd5(const std::uint8_t* data, std::size_t data_size,
                        const std::uint8_t* key, std::size_t key_size,
                        std::uint8_t* hmac_out) {
        if (!data || !key || !hmac_out) {
            return false;
        }

        if (key_size != MD5_HASH_SIZE) {
            return false;
        }

        // OpenSSL 1.1.1 / 3.x compatible HMAC-MD5 (avoid EVP_MAC which is 3.0+ only).
        unsigned int out_len = 0;
        const unsigned char* digest = HMAC(
            EVP_md5(),
            key,
            static_cast<int>(key_size),
            data,
            data_size,
            hmac_out,
            &out_len);
        return digest != nullptr && out_len == MD5_HASH_SIZE;
    }

    bool sendBufferAppendHmac(std::uint8_t* buffer, std::size_t written_size, const std::uint8_t* key, std::size_t key_size, std::uint8_t* hmac_output, std::size_t& new_written_size){
        if (!computeHmacMd5(buffer, written_size, key, key_size, hmac_output)){
            return false;
        }

        //Append the HMAC to the buffer
        std::uint8_t* end_of_buffer = buffer + written_size;
        std::memcpy(end_of_buffer, hmac_output, key_size);

        new_written_size = written_size + key_size;
        return true;
    }

    bool receiveBufferCheckHmac(const std::uint8_t* buffer, std::size_t written_size, const std::uint8_t* key, std::size_t key_size){
        if (written_size < key_size){
            return false;
        }

        const std::uint8_t* received_hmac = buffer + written_size - key_size;
        std::uint8_t computed_hmac[MD5_HASH_SIZE];
        if (key_size > MD5_HASH_SIZE) {
            return false;
        }
        if (!computeHmacMd5(buffer, written_size - key_size, key, key_size, computed_hmac)){
            return false;
        }

        if (memcmp(received_hmac, computed_hmac, key_size) != 0){
            return false;
        }
        return true;
    }
}  // namespace robot::platform::crypto
