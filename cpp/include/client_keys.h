#ifndef CLIENT_KEYS_H
#define CLIENT_KEYS_H

#include <cstdint>
#include <cstddef>
#include <string>

namespace robot::platform {
    enum class ClientPriority {
        kStandard,
        kEmbeddedGUI,
        kPhysicalButton,
        kHighestPriority, // Highest priority
    };

    struct ClientConfig {
        std::string client_name;
        ClientPriority priority;
        std::string hmac_key; // 16-byte key as hex string
    };

    /** Single registry entry for loadClientKeys(). client_id 0 is the multicast key. */
    struct ClientKeyEntry {
        uint16_t client_id;
        const char* name;
        ClientPriority priority;
        const char* hmac_key_hex;
    };

    /**
     * Load client HMAC registry from JSON (YAML-compatible).
     * Expected shape:
     * {
     *   "client_registry": {
     *     "0": { "name": "...", "priority": "kHighestPriority", "hmac_key": "..." },
     *     "10005": { "name": "Panel", "priority": "kStandard", "hmac_key": "..." }
     *   }
     * }
     * Client id 0 is treated as the multicast HMAC key.
     * Must be called once before getClientHmacKey / getClientConfig.
     */
    bool loadClientKeys(const std::string& json_path);

    /** Populate the client key registry from caller-supplied entries. Replaces any prior load. */
    bool loadClientKeys(const ClientKeyEntry* entries, std::size_t count);

    bool isClientKeysLoaded();

    const std::string* getMulticastHmacKey();
    const std::string* getClientHmacKey(std::uint16_t client_id);
    const ClientConfig* getClientConfig(std::uint16_t client_id);
    uint32_t generateSecureSessionId();
} // namespace robot::platform
#endif
