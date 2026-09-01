#ifndef PROTOCOL_EXTENSIONS_H
#define PROTOCOL_EXTENSIONS_H

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <type_traits>

namespace robot::platform::protocol {

constexpr std::uint32_t kExtensionFieldMagic = 0x31545845U; // "EXT1"

constexpr std::size_t kExtensionFieldHeaderSize =
    sizeof(std::uint32_t) + sizeof(std::uint16_t) + sizeof(std::uint16_t);

template <typename T>
bool appendExtensionField(
    std::uint8_t* buffer,
    std::size_t capacity,
    std::size_t& written_size,
    std::uint16_t field_id,
    const T& value) {
    static_assert(std::is_trivially_copyable_v<T>,
                  "protocol extension values must be trivially copyable");
    static_assert(sizeof(T) <= std::numeric_limits<std::uint16_t>::max(),
                  "protocol extension value is too large");

    const std::uint16_t value_size = static_cast<std::uint16_t>(sizeof(T));
    const std::size_t required = kExtensionFieldHeaderSize + sizeof(T);
    if (buffer == nullptr || written_size > capacity || required > capacity - written_size) {
        return false;
    }
    std::memcpy(buffer + written_size, &kExtensionFieldMagic, sizeof(kExtensionFieldMagic));
    written_size += sizeof(kExtensionFieldMagic);
    std::memcpy(buffer + written_size, &field_id, sizeof(field_id));
    written_size += sizeof(field_id);
    std::memcpy(buffer + written_size, &value_size, sizeof(value_size));
    written_size += sizeof(value_size);
    std::memcpy(buffer + written_size, &value, sizeof(T));
    written_size += sizeof(T);
    return true;
}

template <typename T>
bool readExtensionField(
    const std::uint8_t* buffer,
    std::size_t body_size,
    std::size_t extension_offset,
    std::uint16_t field_id,
    T& value) {
    static_assert(std::is_trivially_copyable_v<T>,
                  "protocol extension values must be trivially copyable");
    if (buffer == nullptr || extension_offset > body_size) {
        return false;
    }

    std::size_t cursor = extension_offset;
    while (body_size - cursor >= kExtensionFieldHeaderSize) {
        std::uint32_t magic = 0;
        std::uint16_t current_field_id = 0;
        std::uint16_t value_size = 0;
        std::memcpy(&magic, buffer + cursor, sizeof(magic));
        cursor += sizeof(magic);
        std::memcpy(&current_field_id, buffer + cursor, sizeof(current_field_id));
        cursor += sizeof(current_field_id);
        std::memcpy(&value_size, buffer + cursor, sizeof(value_size));
        cursor += sizeof(value_size);
        if (magic != kExtensionFieldMagic) {
            return false;
        }
        if (value_size > body_size - cursor) {
            return false;
        }
        if (current_field_id == field_id) {
            if (value_size != sizeof(T)) {
                return false;
            }
            std::memcpy(&value, buffer + cursor, sizeof(T));
            return true;
        }
        cursor += value_size;
    }
    return false;
}

}  // namespace robot::platform::protocol

#endif
