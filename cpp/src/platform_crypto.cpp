#include "platform_crypto.h"

#include <openssl/evp.h>
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

        if (key_size != MD5_HASH_SIZE){
            return false;
        }

        // 1. Fetch the HMAC algorithm implementation
        EVP_MAC* mac = EVP_MAC_fetch(nullptr, "HMAC", nullptr);
        if (!mac) return false;

        // 2. Create the MAC execution context
        EVP_MAC_CTX* ctx = EVP_MAC_CTX_new(mac);
        EVP_MAC_free(mac); // We can safely free the base object once the context holds it
        if (!ctx) return false;

        // 3. Configure the underlying hash engine to use MD5
        OSSL_PARAM params[2];
        params[0] = OSSL_PARAM_construct_utf8_string("digest", const_cast<char*>("MD5"), 0);
        params[1] = OSSL_PARAM_construct_end();

        // 4. Initialize the MAC engine with your key and configuration parameters
        if (EVP_MAC_init(ctx, key, key_size, params) != 1) {
            EVP_MAC_CTX_free(ctx);
            return false;
        }

        // 5. Stream the robot packet data into the hashing engine
        if (EVP_MAC_update(ctx, data, data_size) != 1) {
            EVP_MAC_CTX_free(ctx);
            return false;
        }

        // 6. Finalize and extract the 16-byte HMAC signature
        std::size_t out_len = 0;
        if (EVP_MAC_final(ctx, hmac_out, &out_len, MD5_HASH_SIZE) != 1) {
            EVP_MAC_CTX_free(ctx);
            return false;
        }

        EVP_MAC_CTX_free(ctx);
        return true;
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
        std::uint8_t computed_hmac[key_size];
        if (!computeHmacMd5(buffer, written_size - key_size, key, key_size, computed_hmac)){
            return false;
        }

        if (memcmp(received_hmac, computed_hmac, HMAC_KEY_SIZE) != 0){
            return false;
        }
        return true;
    }
}  // namespace robot::platform::crypto