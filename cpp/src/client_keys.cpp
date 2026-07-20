#include "client_keys.h"

#include <iostream>
#include <mutex>
#include <stdexcept>
#include <unordered_map>

#include <openssl/rand.h>
#include <yaml-cpp/yaml.h>

namespace robot::platform {
namespace {

struct ClientKeyStore {
    std::string multicast_hmac_key;
    std::unordered_map<uint16_t, ClientConfig> registry;
    bool loaded = false;
};

ClientKeyStore& keyStore() {
    static ClientKeyStore store;
    return store;
}

std::mutex& keyStoreMutex() {
    static std::mutex mutex;
    return mutex;
}

ClientPriority parsePriority(const std::string& priority) {
    if (priority == "kStandard") return ClientPriority::kStandard;
    if (priority == "kEmbeddedGUI") return ClientPriority::kEmbeddedGUI;
    if (priority == "kPhysicalButton") return ClientPriority::kPhysicalButton;
    if (priority == "kHighestPriority") return ClientPriority::kHighestPriority;
    throw std::runtime_error("Unknown client priority: " + priority);
}

} // namespace

bool loadClientKeys(const std::string& json_path) {
    std::lock_guard<std::mutex> lock(keyStoreMutex());
    try {
        YAML::Node root = YAML::LoadFile(json_path);
        if (!root || !root["client_registry"]) {
            std::cerr << "loadClientKeys: missing client_registry in " << json_path << std::endl;
            return false;
        }

        ClientKeyStore next;
        const YAML::Node registry = root["client_registry"];
        for (const auto& entry : registry) {
            const uint16_t client_id = entry.first.as<uint16_t>();
            const YAML::Node& cfg = entry.second;
            if (!cfg["name"] || !cfg["priority"] || !cfg["hmac_key"]) {
                std::cerr << "loadClientKeys: incomplete entry for client_id " << client_id << std::endl;
                return false;
            }

            ClientConfig config;
            config.client_name = cfg["name"].as<std::string>();
            config.priority = parsePriority(cfg["priority"].as<std::string>());
            config.hmac_key = cfg["hmac_key"].as<std::string>();
            if (config.hmac_key.empty()) {
                std::cerr << "loadClientKeys: empty hmac_key for client_id " << client_id << std::endl;
                return false;
            }

            if (client_id == 0) {
                next.multicast_hmac_key = config.hmac_key;
            }
            next.registry.emplace(client_id, std::move(config));
        }

        if (next.multicast_hmac_key.empty()) {
            std::cerr << "loadClientKeys: missing multicast key (client_id 0) in " << json_path << std::endl;
            return false;
        }

        next.loaded = true;
        keyStore() = std::move(next);
        std::cout << "Loaded client keys from " << json_path
                  << " (" << keyStore().registry.size() << " entries)\n";
        return true;
    } catch (const std::exception& e) {
        std::cerr << "loadClientKeys failed for " << json_path << ": " << e.what() << std::endl;
        return false;
    }
}

bool isClientKeysLoaded() {
    std::lock_guard<std::mutex> lock(keyStoreMutex());
    return keyStore().loaded;
}

const std::string* getMulticastHmacKey() {
    std::lock_guard<std::mutex> lock(keyStoreMutex());
    if (!keyStore().loaded) {
        return nullptr;
    }
    return &keyStore().multicast_hmac_key;
}

const std::string* getClientHmacKey(std::uint16_t client_id) {
    if (client_id == 0) {
        return getMulticastHmacKey();
    }

    std::lock_guard<std::mutex> lock(keyStoreMutex());
    if (!keyStore().loaded) {
        return nullptr;
    }
    auto it = keyStore().registry.find(client_id);
    if (it != keyStore().registry.end()) {
        return &(it->second.hmac_key);
    }
    return nullptr;
}

const ClientConfig* getClientConfig(std::uint16_t client_id) {
    std::lock_guard<std::mutex> lock(keyStoreMutex());
    if (!keyStore().loaded) {
        return nullptr;
    }
    auto it = keyStore().registry.find(client_id);
    if (it != keyStore().registry.end()) {
        return &(it->second);
    }
    return nullptr;
}

uint32_t generateSecureSessionId() {
    uint32_t session_id = 0;

    if (RAND_bytes(reinterpret_cast<uint8_t*>(&session_id), sizeof(session_id)) != 1) {
        throw std::runtime_error("Security Error: Failed to generate secure random bytes.");
    }

    while (session_id == 0) {
        if (RAND_bytes(reinterpret_cast<uint8_t*>(&session_id), sizeof(session_id)) != 1) {
            throw std::runtime_error("Security Error: Failed to generate secure random bytes.");
        }
    }

    return session_id;
}
} // namespace robot::platform
