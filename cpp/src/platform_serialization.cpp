#include "platform_serialization.h"

#include <iostream>
#include <cstring>
#include <limits>
#include <variant>
#include <type_traits>
#include <memory>

namespace robot::platform::serialization {
    namespace {
        template <typename T>
        void writeRaw(const T& src, std::uint8_t*& cursor, std::size_t& remaining) {
            if (remaining < sizeof(T)) {
                throw std::runtime_error("writeRaw: buffer overflow.");
            }

            static_assert(!std::is_array_v<T>, "Use the array overload for raw arrays!");
            static_assert(std::is_trivially_copyable_v<T>, "writeRaw requires trivially copyable type");
            std::memcpy(cursor, &src, sizeof(T));
            cursor += sizeof(T);
            remaining -= sizeof(T);
        }

        template <typename T, std::size_t N>
        void writeRaw(const T (&src)[N], std::size_t count, std::uint8_t*& cursor, std::size_t& remaining) {
            const std::size_t total_bytes = count * sizeof(T);
            if (remaining < total_bytes) {
                throw std::runtime_error("writeRaw array: buffer overflow.");
            }
            static_assert(std::is_trivially_copyable_v<T>, "writeRaw requires trivially copyable type");
            std::memcpy(cursor, &src, total_bytes);
            cursor += total_bytes;
            remaining -= total_bytes;
        }

        template <typename EnumT>
        void writeEnum(const EnumT& src, std::uint8_t*& cursor, std::size_t& remaining) {
            static_assert(std::is_enum_v<EnumT>, "writeEnum requires enum type");
            using RawT = std::underlying_type_t<EnumT>;
            RawT raw_val = static_cast<RawT>(src);
            writeRaw(raw_val, cursor, remaining);
        }

        template <typename T>
        void readRaw(const std::uint8_t*& cursor, T& dst) {
            static_assert(std::is_trivially_copyable_v<T>, "readRaw requires trivially copyable type");
            static_assert(!std::is_array_v<T>, "Use the array overload for raw arrays!");
            std::memcpy(&dst, cursor, sizeof(T)); // Internal implementation takes the address
            cursor += sizeof(T);
        }

        template <typename T, std::size_t N>
        void readRaw(const std::uint8_t*& cursor, T (&dst)[N], std::size_t count) {
            static_assert(std::is_trivially_copyable_v<T>, "readRaw array requires trivially copyable element type");
            
            if (count > N) {
                throw std::runtime_error("readRaw array overflow: count exceeds array capacity.");
            }
            
            const std::size_t total_bytes = count * sizeof(T);
            std::memcpy(dst, cursor, total_bytes);
            cursor += total_bytes;
        }

        template <typename EnumT>
        void readEnum(const std::uint8_t*& cursor, EnumT& dst) { 
            static_assert(std::is_enum_v<EnumT>, "readEnum requires enum type");
            
            using RawT = std::underlying_type_t<EnumT>;
            RawT raw_val;
            readRaw(cursor, raw_val); 
            dst = static_cast<EnumT>(raw_val);
        }

        void writePositionTarget(const PositionTarget& src, std::uint8_t*& cursor, std::size_t& remaining) {
            writeRaw(src.x, cursor, remaining);
            writeRaw(src.y, cursor, remaining);
            writeRaw(src.z, cursor, remaining);
        }

        void readPositionTarget(const std::uint8_t*& cursor, PositionTarget& dst) {
            readRaw(cursor, dst.x);
            readRaw(cursor, dst.y);
            readRaw(cursor, dst.z);
        }

        void writeCartesianTarget(const CartesianTarget& src, std::uint8_t*& cursor, std::size_t& remaining) {
            writeRaw(src.x, cursor, remaining);
            writeRaw(src.y, cursor, remaining);
            writeRaw(src.z, cursor, remaining);
            writeRaw(src.qw, cursor, remaining);
            writeRaw(src.qx, cursor, remaining);
            writeRaw(src.qy, cursor, remaining);
            writeRaw(src.qz, cursor, remaining);
        }

        void readCartesianTarget(const std::uint8_t*& cursor, CartesianTarget& dst) {
            readRaw(cursor, dst.x);
            readRaw(cursor, dst.y);
            readRaw(cursor, dst.z);
            readRaw(cursor, dst.qw);
            readRaw(cursor, dst.qx);
            readRaw(cursor, dst.qy);
            readRaw(cursor, dst.qz);
        }

        void writeCartesianArray(const CartesianTarget* src, std::uint8_t*& cursor, std::size_t& remaining, std::size_t count) {
            for (uint32_t i = 0; i < count; ++i) {
                writeCartesianTarget(src[i], cursor, remaining);
            }
        }

        void readCartesianArray(const std::uint8_t*& cursor, CartesianTarget* dst, std::size_t count) {
            for (uint32_t i = 0; i < count; ++i) {
                readCartesianTarget(cursor, dst[i]);
            }
        }

        void writeArmUnion(const SinglePointTarget& src, std::uint8_t*& cursor, std::size_t& remaining,
            ControlStrategy strategy, uint8_t enable_jog,
            std::uint8_t arm_size, const std::uint8_t* arm_joint_size) {
            if (enable_jog) {
                if (strategy == ControlStrategy::kCartesianLine || strategy == ControlStrategy::kCartesian) {
                    for (uint32_t i = 0; i < arm_size; ++i) {
                        writeRaw(src.arm_cartesian_jog[i], 6, cursor, remaining);
                    }
                } else {
                    for (uint32_t i = 0; i < arm_size; ++i) {
                        writeRaw(src.arm_joint_jog[i], arm_joint_size[i], cursor, remaining);
                    }
                }
            } else if (strategy == ControlStrategy::kCartesianLine || strategy == ControlStrategy::kCartesian) {
                writeCartesianArray(src.arm_tool_cartesian, cursor, remaining, arm_size);
            } else {
                for (uint32_t i = 0; i < arm_size; ++i) {
                    writeRaw(src.arm_joint[i], arm_joint_size[i], cursor, remaining);
                }
            }
        }

        void readArmUnion(const std::uint8_t*& cursor, SinglePointTarget& dst,
            ControlStrategy strategy, uint8_t enable_jog,
            std::uint8_t arm_size, const std::uint8_t* arm_joint_size) {
            std::memset(&dst.arm_joint, 0, sizeof(dst.arm_joint));
            if (enable_jog) {
                if (strategy == ControlStrategy::kCartesianLine || strategy == ControlStrategy::kCartesian) {
                    for (uint32_t i = 0; i < arm_size; ++i) {
                        readRaw(cursor, dst.arm_cartesian_jog[i], 6);
                    }
                } else {
                    for (uint32_t i = 0; i < arm_size; ++i) {
                        readRaw(cursor, dst.arm_joint_jog[i], arm_joint_size[i]);
                    }
                }
            } else if (strategy == ControlStrategy::kCartesianLine || strategy == ControlStrategy::kCartesian) {
                readCartesianArray(cursor, dst.arm_tool_cartesian, arm_size);
            } else {
                for (uint32_t i = 0; i < arm_size; ++i) {
                    readRaw(cursor, dst.arm_joint[i], arm_joint_size[i]);
                }
            }
        }

        void writeSinglePointTarget(const SinglePointTarget& src, std::uint8_t*& cursor, std::size_t& remaining, 
            ControlStrategy strategy, uint8_t enable_jog,
            std::uint8_t arm_size, const std::uint8_t* arm_joint_size, std::uint8_t gripper_size, const std::uint8_t* gripper_joint_size) {
            writeRaw(src.interpolation_t, cursor, remaining);
            writeRaw(src.interpolation_speed_ratio, cursor, remaining);
            for (uint32_t i = 0; i < gripper_size; ++i) {
                writeRaw(src.gripper_joint[i], gripper_joint_size[i], cursor, remaining);
            }
            writeArmUnion(src, cursor, remaining, strategy, enable_jog, arm_size, arm_joint_size);
        }

        void readSinglePointTarget(const std::uint8_t*& cursor, SinglePointTarget& dst, 
            ControlStrategy strategy, uint8_t enable_jog,
            std::uint8_t arm_size, const std::uint8_t* arm_joint_size, std::uint8_t gripper_size, const std::uint8_t* gripper_joint_size) {
            readRaw(cursor, dst.interpolation_t);
            readRaw(cursor, dst.interpolation_speed_ratio);
            for (uint32_t i = 0; i < gripper_size; ++i) {
                readRaw(cursor, dst.gripper_joint[i], gripper_joint_size[i]);
            }
            readArmUnion(cursor, dst, strategy, enable_jog, arm_size, arm_joint_size);
        }

        void writeSinglePointArray(const SinglePointTarget* src, std::uint8_t*& cursor, std::size_t& remaining, 
            uint32_t count, ControlStrategy strategy, uint8_t enable_jog,
            std::uint8_t arm_size, const std::uint8_t* arm_joint_size, std::uint8_t gripper_size, const std::uint8_t* gripper_joint_size) {
            for (uint32_t i = 0; i < count; ++i) {
                writeSinglePointTarget(src[i], cursor, remaining, strategy, enable_jog, arm_size, arm_joint_size, gripper_size, gripper_joint_size);
            }
        }

        void readSinglePointArray(const std::uint8_t*& cursor, SinglePointTarget* dst, 
            uint32_t count, ControlStrategy strategy, uint8_t enable_jog,
            std::uint8_t arm_size, const std::uint8_t* arm_joint_size, std::uint8_t gripper_size, const std::uint8_t* gripper_joint_size) {
            for (uint32_t i = 0; i < count; ++i) {
                readSinglePointTarget(cursor, dst[i], strategy, enable_jog, arm_size, arm_joint_size, gripper_size, gripper_joint_size);
            }
        }

        template<typename T>
        void catchToBytesError(const std::string& function_name, const std::exception& e, 
            const T& value, const std::size_t buffer_size, const std::size_t& written_size,
            const std::vector<std::pair<std::string, std::uint8_t>>& phase_size) {
            std::cerr << function_name << " " << e.what() << std::endl;
            std::cout << "\tWrote bytes: " << written_size <<", buffer bytes: " << buffer_size << std::endl;
            for (const auto& phase : phase_size) {
                std::cout <<"\t\t"<< phase.first << ": " << static_cast<std::size_t>(phase.second) << " bytes" << std::endl;
            }
        }

        bool verifyReadBuffer(const std::string& function_name, const std::uint8_t* cursor,
            const std::uint8_t* buffer, std::size_t buffer_size,
            const std::vector<std::pair<std::string, std::uint8_t>>& phase_size) {
            if (cursor != buffer + buffer_size) {
                std::cout << function_name << " read buffer mismatch" << std::endl;
                std::cout << "\t Read bytes: " << cursor - buffer << ", Correct bytes should be: " << buffer_size << std::endl;
                for (const auto& phase : phase_size) {
                    std::cout <<"\t\t"<< phase.first << ": " << static_cast<std::size_t>(phase.second) << " bytes" << std::endl;
                }
                return false;
            }
            return true;
        }
    }  // namespace

    bool getReceivedMessageType(const std::uint8_t* buffer, std::size_t buffer_size, MessageType& message_type){
        uint32_t magic_header;
        if (buffer_size < sizeof(magic_header) + sizeof(MessageType)) {
            return false;
        }
        const std::uint8_t* cursor = buffer + sizeof(magic_header);
        readEnum(cursor, message_type);
        return true;
    }

    bool getReceivedHeader(const std::uint8_t* buffer, std::size_t buffer_size, uint32_t& magic_header, MessageType& message_type, uint16_t& client_id){
        if (buffer_size < sizeof(magic_header) + sizeof(message_type) + sizeof(client_id)) {
            return false;
        }
        
        const std::uint8_t* cursor = buffer;
        try{
            readRaw(cursor, magic_header);
            readEnum(cursor, message_type);
            readRaw(cursor, client_id);
        }catch (const std::exception& e) {
            std::cerr << "Error getting received header: " << e.what() << std::endl;
            return false;
        }
        return true;
    }

    bool toBytes(const SdkCommandReq& value, std::uint8_t* buffer, std::size_t buffer_size, std::size_t& written_size) {
        std::uint8_t* cursor = buffer;
        std::size_t remaining = buffer_size;
        std::vector<std::pair<std::string, std::uint8_t>> write_phase;
        
        try{
            writeRaw(value.magic_header, cursor, remaining);
            writeEnum(MessageType::kSdkCommandReq, cursor, remaining);
            writeRaw(value.client_id, cursor, remaining);

            writeRaw(value.session_id, cursor, remaining);
            writeRaw(value.sequence_id, cursor, remaining);
            writeRaw(value.timestamp_us, cursor, remaining);
            write_phase.push_back(std::make_pair("after timestamp_us", cursor - buffer));

            // --- Payload Block ---
            writeRaw(value.payload.robot_name, MAX_NAME_SIZE, cursor, remaining);
            writeRaw(value.payload.arm_size, cursor, remaining);
            writeRaw(value.payload.gripper_size, cursor, remaining);
            writeRaw(value.payload.arm_joint_size, value.payload.arm_size, cursor, remaining);
            writeRaw(value.payload.gripper_joint_size, value.payload.gripper_size, cursor, remaining);
            writeEnum(value.payload.activated_control_strategy, cursor, remaining);
            writeRaw(value.payload.enable_jog, cursor, remaining);
            writeRaw(value.payload.target_count, cursor, remaining);
            write_phase.push_back(std::make_pair("after target_count", cursor - buffer));

            for (uint32_t i = 0; i < value.payload.target_count; ++i) {
                writeSinglePointTarget(value.payload.target[i], cursor, remaining,
                    value.payload.activated_control_strategy, value.payload.enable_jog,
                    value.payload.arm_size, value.payload.arm_joint_size, 
                    value.payload.gripper_size, value.payload.gripper_joint_size);
            }
        } catch (const std::exception& e) {
            catchToBytesError("catchToBytesError:toBytes(SdkCommandReq)", e, value, buffer_size, written_size, write_phase);
            return false;
        }
        written_size = cursor - buffer;
        return true;
    }

    bool toBytes(const SdkCommandRes& value, std::uint8_t* buffer, std::size_t buffer_size, std::size_t& written_size) {
        std::uint8_t* cursor = buffer;
        std::size_t remaining = buffer_size;
        std::vector<std::pair<std::string, std::uint8_t>> write_phase;

        try{
            writeRaw(value.magic_header, cursor, remaining);
            writeEnum(MessageType::kSdkCommandRes, cursor, remaining);
            writeRaw(value.request_client_id, cursor, remaining);

            writeRaw(value.request_sequence_id, cursor, remaining);
            writeRaw(value.request_received_us, cursor, remaining);
            writeRaw(value.response_sent_us, cursor, remaining);
            write_phase.push_back(std::make_pair("after response_sent_us", cursor - buffer));

            writeEnum(value.payload.status, cursor, remaining);
        } catch (const std::exception& e) {
            catchToBytesError("catchToBytesError:toBytes(SdkCommandRes)", e, value, buffer_size, written_size, write_phase);
            return false;
        }
        written_size = cursor - buffer;
        return true;
    }

    bool toBytes(const SdkConfigReq& value, std::uint8_t* buffer, std::size_t buffer_size, std::size_t& written_size) {
        std::uint8_t* cursor = buffer;
        std::size_t remaining = buffer_size;
        std::vector<std::pair<std::string, std::uint8_t>> write_phase;

        try{
            writeRaw(value.magic_header, cursor, remaining);
            writeEnum(MessageType::kSdkConfigReq, cursor, remaining);
            writeRaw(value.client_id, cursor, remaining);

            writeRaw(value.session_id, cursor, remaining);
            writeRaw(value.sequence_id, cursor, remaining);
            writeRaw(value.timestamp_us, cursor, remaining);
            write_phase.push_back(std::make_pair("after timestamp_us", cursor - buffer));

            writeRaw(value.payload.robot_name, MAX_NAME_SIZE, cursor, remaining);
            writeRaw(value.payload.simulation, cursor, remaining);
            writeRaw(value.payload.read_only, cursor, remaining);
            writeRaw(value.payload.arm_size, cursor, remaining);
            writeRaw(value.payload.gripper_size, cursor, remaining);
            writeRaw(value.payload.arm_joint_size, value.payload.arm_size, cursor, remaining);
            writeRaw(value.payload.gripper_joint_size, value.payload.gripper_size, cursor, remaining);
            write_phase.push_back(std::make_pair("after gripper_joint_size", cursor - buffer));

            for (uint32_t i = 0; i < value.payload.arm_size; ++i) {
                writeEnum(value.payload.arm[i].control_strategy, cursor, remaining);
                writeEnum(value.payload.arm[i].filter_type, cursor, remaining);
                writeEnum(value.payload.arm[i].target_type, cursor, remaining);
                writeEnum(value.payload.arm[i].actuator_mode, cursor, remaining);
                writeEnum(value.payload.arm[i].playback_cmd, cursor, remaining);
                writeEnum(value.payload.arm[i].frame_reference, cursor, remaining);
                writeRaw(value.payload.arm[i].reset_control_mem, cursor, remaining);
                writeRaw(value.payload.arm[i].reset_interpolation, cursor, remaining);
                write_phase.push_back(std::make_pair("after arm[" + std::to_string(i) + "]'s reset_interpolation", cursor - buffer));
                
                writeRaw(value.payload.arm[i].enable_joint, value.payload.arm_joint_size[i], cursor, remaining);
                writePositionTarget(value.payload.arm[i].tool_offset, cursor, remaining);
                writeRaw(value.payload.arm[i].soft_limit_position, value.payload.arm_joint_size[i], cursor, remaining);
                writeRaw(value.payload.arm[i].soft_limit_velocity, value.payload.arm_joint_size[i], cursor, remaining);
                writeRaw(value.payload.arm[i].soft_limit_torque, value.payload.arm_joint_size[i], cursor, remaining);
                write_phase.push_back(std::make_pair("after arm[" + std::to_string(i) + "]'s soft_limit_torque", cursor - buffer));    
            }

            for (uint32_t i = 0; i < value.payload.gripper_size; ++i) {
                writeRaw(value.payload.gripper[i].target_type, cursor, remaining);
                writeRaw(value.payload.gripper[i].soft_limit_position, value.payload.gripper_joint_size[i], cursor, remaining);
                write_phase.push_back(std::make_pair("after gripper[" + std::to_string(i) + "]'s soft_limit_position", cursor - buffer));
            }
        } catch (const std::exception& e) {
            catchToBytesError("catchToBytesError:toBytes(SdkConfigReq)", e, value, buffer_size, written_size, write_phase);
            return false;
        }
        written_size = cursor - buffer;
        return true;
    }

    bool toBytes(const SdkConfigRes& value, std::uint8_t* buffer, std::size_t buffer_size, std::size_t& written_size) {
        std::uint8_t* cursor = buffer;
        std::size_t remaining = buffer_size;
        std::vector<std::pair<std::string, std::uint8_t>> write_phase;

        try{
            writeRaw(value.magic_header, cursor, remaining);
            writeEnum(MessageType::kSdkConfigRes, cursor, remaining);
            writeRaw(value.request_client_id, cursor, remaining);

            writeRaw(value.request_sequence_id, cursor, remaining);
            writeRaw(value.request_received_us, cursor, remaining);
            writeRaw(value.response_sent_us, cursor, remaining);
            write_phase.push_back(std::make_pair("after response_sent_us", cursor - buffer));

            writeEnum(value.payload.status, cursor, remaining);
            writeRaw(value.payload.robot_name, MAX_NAME_SIZE, cursor, remaining);
            writeRaw(value.payload.simulation, cursor, remaining);
            writeRaw(value.payload.arm_size, cursor, remaining);
            writeRaw(value.payload.gripper_size, cursor, remaining);
            writeRaw(value.payload.arm_joint_size, value.payload.arm_size, cursor, remaining);
            writeRaw(value.payload.gripper_joint_size, value.payload.gripper_size, cursor, remaining);
            write_phase.push_back(std::make_pair("after gripper_joint_size", cursor - buffer));

            for (uint32_t i = 0; i < value.payload.arm_size; ++i) {
                writeRaw(value.payload.arm[i].name, MAX_NAME_SIZE, cursor, remaining);
                writeEnum(value.payload.arm[i].control_strategy, cursor, remaining);
                writeEnum(value.payload.arm[i].filter_type, cursor, remaining);
                writeEnum(value.payload.arm[i].target_type, cursor, remaining);
                writeEnum(value.payload.arm[i].actuator_mode, cursor, remaining);
                writeEnum(value.payload.arm[i].playback_cmd, cursor, remaining);
                writeEnum(value.payload.arm[i].frame_reference, cursor, remaining);
                write_phase.push_back(std::make_pair("after arm[" + std::to_string(i) + "]'s frame_reference", cursor - buffer));
                
                writePositionTarget(value.payload.arm[i].tool_offset, cursor, remaining);
                writeRaw(value.payload.arm[i].enabled_joint, value.payload.arm_joint_size[i], cursor, remaining);
                writeRaw(value.payload.arm[i].soft_limit_position, value.payload.arm_joint_size[i], cursor, remaining);
                writeRaw(value.payload.arm[i].soft_limit_velocity, value.payload.arm_joint_size[i], cursor, remaining);
                writeRaw(value.payload.arm[i].soft_limit_torque, value.payload.arm_joint_size[i], cursor, remaining);
                writeRaw(value.payload.arm[i].hard_limit_position, value.payload.arm_joint_size[i], cursor, remaining);
                writeRaw(value.payload.arm[i].hard_limit_velocity, value.payload.arm_joint_size[i], cursor, remaining);
                writeRaw(value.payload.arm[i].hard_limit_torque, value.payload.arm_joint_size[i], cursor, remaining);
                writeRaw(value.payload.arm[i].follow_limit_position, value.payload.arm_joint_size[i], cursor, remaining);
                writeRaw(value.payload.arm[i].follow_limit_velocity, value.payload.arm_joint_size[i], cursor, remaining);
                writeRaw(value.payload.arm[i].follow_limit_torque, value.payload.arm_joint_size[i], cursor, remaining);
                writeRaw(value.payload.arm[i].jump_limit_position, value.payload.arm_joint_size[i], cursor, remaining);
                writeRaw(value.payload.arm[i].jump_limit_velocity, value.payload.arm_joint_size[i], cursor, remaining);
                writeRaw(value.payload.arm[i].jump_limit_torque, value.payload.arm_joint_size[i], cursor, remaining);
                write_phase.push_back(std::make_pair("after arm[" + std::to_string(i) + "]'s jump_limit_torque", cursor - buffer));
            }

            for (uint32_t i = 0; i < value.payload.gripper_size; ++i) {
                writeRaw(value.payload.gripper[i].name, MAX_NAME_SIZE, cursor, remaining);
                writeEnum(value.payload.gripper[i].target_type, cursor, remaining);
                writeRaw(value.payload.gripper[i].soft_limit_position, value.payload.gripper_joint_size[i], cursor, remaining);
                writeRaw(value.payload.gripper[i].hard_limit_position, value.payload.gripper_joint_size[i], cursor, remaining);
                write_phase.push_back(std::make_pair("after gripper[" + std::to_string(i) + "]'s soft_limit_position", cursor - buffer));
            }
        } catch (const std::exception& e) {
            catchToBytesError("catchToBytesError:toBytes(SdkConfigRes)", e, value, buffer_size, written_size, write_phase);
            return false;
        }
        written_size = cursor - buffer;
        return true;
    }

    bool toBytes(const SdkHeartbeatReq& value, std::uint8_t* buffer, std::size_t buffer_size, std::size_t& written_size) {
        std::uint8_t* cursor = buffer;
        std::size_t remaining = buffer_size;
        std::vector<std::pair<std::string, std::uint8_t>> write_phase;

        try{
            writeRaw(value.magic_header, cursor, remaining);
            writeEnum(MessageType::kSdkHeartbeatReq, cursor, remaining);
            writeRaw(value.client_id, cursor, remaining);
            write_phase.push_back(std::make_pair("after client_id", cursor - buffer));

            writeRaw(value.session_id, cursor, remaining);
            writeRaw(value.timestamp_us, cursor, remaining);
            write_phase.push_back(std::make_pair("after timestamp_us", cursor - buffer));
        }catch (const std::exception& e) {
            catchToBytesError("catchToBytesError:toBytes(SdkHeartbeatReq)", e, value, buffer_size, written_size, write_phase);
            return false;
        }
        written_size = cursor - buffer;
        return true;
    }

    bool toBytes(const SdkSafeguardReq& value, std::uint8_t* buffer, std::size_t buffer_size, std::size_t& written_size) {
        std::uint8_t* cursor = buffer;
        std::size_t remaining = buffer_size;
        std::vector<std::pair<std::string, std::uint8_t>> write_phase;

        try{
            writeRaw(value.magic_header, cursor, remaining);
            writeEnum(MessageType::kSdkSafeguardReq, cursor, remaining);
            writeRaw(value.client_id, cursor, remaining);

            writeRaw(value.session_id, cursor, remaining);
            writeRaw(value.timestamp_us, cursor, remaining);
            write_phase.push_back(std::make_pair("after timestamp_us", cursor - buffer));
        }catch (const std::exception& e) {
            catchToBytesError("catchToBytesError:toBytes(SdkSafeguardReq)", e, value, buffer_size, written_size, write_phase);
            return false;
        }
        written_size = cursor - buffer;
        return true;
    }

    bool toBytes(const SdkHandshakeReq& value, std::uint8_t* buffer, std::size_t buffer_size, std::size_t& written_size) {
        std::uint8_t* cursor = buffer;
        std::size_t remaining = buffer_size;
        std::vector<std::pair<std::string, std::uint8_t>> write_phase;

        try{
            writeRaw(value.magic_header, cursor, remaining);
            writeEnum(MessageType::kSdkHandshakeReq, cursor, remaining);
            writeRaw(value.client_id, cursor, remaining);

            writeRaw(value.sequence_id, cursor, remaining);
            writeRaw(value.timestamp_us, cursor, remaining);
        }catch (const std::exception& e) {
            catchToBytesError("catchToBytesError:toBytes(SdkHandshakeReq)", e, value, buffer_size, written_size, write_phase);
            return false;
        }

        written_size = cursor - buffer;
        return true;
    }

    bool toBytes(const SdkHandshakeRes& value, std::uint8_t* buffer, std::size_t buffer_size, std::size_t& written_size) {
        std::uint8_t* cursor = buffer;
        std::size_t remaining = buffer_size;
        std::vector<std::pair<std::string, std::uint8_t>> write_phase;

        try{
            writeRaw(value.magic_header, cursor, remaining);
            writeEnum(MessageType::kSdkHandshakeRes, cursor, remaining);
            writeRaw(value.request_client_id, cursor, remaining);
            
            writeRaw(value.request_sequence_id, cursor, remaining);
            writeRaw(value.request_received_us, cursor, remaining);
            writeRaw(value.response_sent_us, cursor, remaining);
            writeRaw(value.assigned_session_id, cursor, remaining);

            writeEnum(value.payload.status, cursor, remaining);
            writeRaw(value.payload.robot_name, MAX_NAME_SIZE, cursor, remaining);
            write_phase.push_back(std::make_pair("after robot_name", cursor - buffer));

        }catch (const std::exception& e) {
            catchToBytesError("catchToBytesError:toBytes(SdkHandshakeRes)", e, value, buffer_size, written_size, write_phase);
            return false;
        }
        written_size = cursor - buffer;
        return true;
    }

    bool toBytes(const SdkReleaseControlReq& value, std::uint8_t* buffer, std::size_t buffer_size, std::size_t& written_size) {
        std::uint8_t* cursor = buffer;
        std::size_t remaining = buffer_size;
        std::vector<std::pair<std::string, std::uint8_t>> write_phase;

        try{
            writeRaw(value.magic_header, cursor, remaining);
            writeEnum(MessageType::kSdkReleaseControlReq, cursor, remaining);
            writeRaw(value.client_id, cursor, remaining);

            writeRaw(value.session_id, cursor, remaining);
            writeRaw(value.sequence_id, cursor, remaining);
            writeRaw(value.timestamp_us, cursor, remaining);
        }catch (const std::exception& e) {
            catchToBytesError("catchToBytesError:toBytes(SdkReleaseControlReq)", e, value, buffer_size, written_size, write_phase);
            return false;
        }

        written_size = cursor - buffer;
        return true;
    }

    bool toBytes(const SdkReleaseControlRes& value, std::uint8_t* buffer, std::size_t buffer_size, std::size_t& written_size) {
        std::uint8_t* cursor = buffer;
        std::size_t remaining = buffer_size;
        std::vector<std::pair<std::string, std::uint8_t>> write_phase;

        try{
            writeRaw(value.magic_header, cursor, remaining);
            writeEnum(MessageType::kSdkReleaseControlRes, cursor, remaining);
            writeRaw(value.request_client_id, cursor, remaining);
            
            writeRaw(value.request_received_us, cursor, remaining);
            writeRaw(value.response_sent_us, cursor, remaining);

            writeEnum(value.payload.status, cursor, remaining);
            write_phase.push_back(std::make_pair("after assigned_session_id", cursor - buffer));

        }catch (const std::exception& e) {
            catchToBytesError("catchToBytesError:toBytes(SdkReleaseControlRes)", e, value, buffer_size, written_size, write_phase);
            return false;
        }
        written_size = cursor - buffer;
        return true;
    }

    bool toBytes(const SdkRecoveryReq& value, std::uint8_t* buffer, std::size_t buffer_size, std::size_t& written_size) {
        std::uint8_t* cursor = buffer;
        std::size_t remaining = buffer_size;
        std::vector<std::pair<std::string, std::uint8_t>> write_phase;

        try{
            writeRaw(value.magic_header, cursor, remaining);
            writeEnum(MessageType::kSdkRecoveryReq, cursor, remaining);
            writeRaw(value.client_id, cursor, remaining);

            writeRaw(value.session_id, cursor, remaining);
            writeRaw(value.sequence_id, cursor, remaining);
            writeRaw(value.timestamp_us, cursor, remaining);
        }catch (const std::exception& e) {
            catchToBytesError("catchToBytesError:toBytes(SdkRecoveryReq)", e, value, buffer_size, written_size, write_phase);
            return false;
        }

        written_size = cursor - buffer;
        return true;
    }

    bool toBytes(const SdkRecoveryRes& value, std::uint8_t* buffer, std::size_t buffer_size, std::size_t& written_size) {
        std::uint8_t* cursor = buffer;
        std::size_t remaining = buffer_size;
        std::vector<std::pair<std::string, std::uint8_t>> write_phase;

        try{
            writeRaw(value.magic_header, cursor, remaining);
            writeEnum(MessageType::kSdkReleaseControlRes, cursor, remaining);
            writeRaw(value.request_client_id, cursor, remaining);
            
            writeRaw(value.request_received_us, cursor, remaining);
            writeRaw(value.response_sent_us, cursor, remaining);

            writeEnum(value.payload.status, cursor, remaining);
            write_phase.push_back(std::make_pair("after assigned_session_id", cursor - buffer));

        }catch (const std::exception& e) {
            catchToBytesError("catchToBytesError:toBytes(SdkRecoveryRes)", e, value, buffer_size, written_size, write_phase);
            return false;
        }
        written_size = cursor - buffer;
        return true;
    }

    bool toBytes(const SrvState& value, std::uint8_t* buffer, std::size_t buffer_size, std::size_t& written_size) {
        std::uint8_t* cursor = buffer;
        std::size_t remaining = buffer_size;
        std::vector<std::pair<std::string, std::uint8_t>> write_phase;

        try{
            writeRaw(value.magic_header, cursor, remaining);
            writeEnum(MessageType::kSrvState, cursor, remaining);
            writeRaw(value.client_id, cursor, remaining);
            
            writeRaw(value.sequence_id, cursor, remaining);
            writeRaw(value.current_client_id, cursor, remaining);
            writeRaw(value.current_client_sequence_id, cursor, remaining);
            writeRaw(value.timestamp_us, cursor, remaining);
            write_phase.push_back(std::make_pair("after timestamp_us", cursor - buffer));

            writeRaw(value.payload.robot_name, MAX_NAME_SIZE, cursor, remaining);
            writeEnum(value.payload.system_state, cursor, remaining);
            writeEnum(value.payload.plan_result, cursor, remaining);
            writeRaw(value.payload.system_diagnostic_flags, cursor, remaining);
            writeRaw(value.payload.session_id, cursor, remaining);
            writeRaw(value.payload.last_processed_seq, cursor, remaining);
            write_phase.push_back(std::make_pair("after last_processed_seq", cursor - buffer));

            writeRaw(value.payload.arm_size, cursor, remaining);
            writeRaw(value.payload.gripper_size, cursor, remaining);
            writeRaw(value.payload.arm_joint_size, value.payload.arm_size, cursor, remaining);
            writeRaw(value.payload.gripper_joint_size, value.payload.gripper_size, cursor, remaining);
            write_phase.push_back(std::make_pair("after gripper_joint_size", cursor - buffer));

            for (uint8_t i = 0; i < value.payload.arm_size; ++i) {
                writeRaw(value.payload.arm[i].position, value.payload.arm_joint_size[i], cursor, remaining);
                writeRaw(value.payload.arm[i].velocity, value.payload.arm_joint_size[i], cursor, remaining);
                writeRaw(value.payload.arm[i].torque, value.payload.arm_joint_size[i], cursor, remaining);
                writeRaw(value.payload.arm[i].acceleration, value.payload.arm_joint_size[i], cursor, remaining);
                writeCartesianArray(value.payload.arm[i].joint_poses, cursor, remaining, value.payload.arm_joint_size[i]);
                writeCartesianTarget(value.payload.arm[i].tool_pose, cursor, remaining);
                writeRaw(value.payload.arm[i].last_set_command, value.payload.arm_joint_size[i], cursor, remaining);
                writeRaw(value.payload.arm[i].diagnostic_flags, value.payload.arm_joint_size[i], cursor, remaining);
                write_phase.push_back(std::make_pair("after arm[" + std::to_string(i) + "]'s last_set_command", cursor - buffer));
            }

            for (uint8_t i = 0; i < value.payload.gripper_size; ++i) {
                writeRaw(value.payload.gripper[i].position, value.payload.gripper_joint_size[i], cursor, remaining);
                writeRaw(value.payload.gripper[i].last_set_command, value.payload.gripper_joint_size[i], cursor, remaining);
                writeRaw(value.payload.gripper[i].diagnostic_flags, value.payload.gripper_joint_size[i], cursor, remaining);
                write_phase.push_back(std::make_pair("after gripper[" + std::to_string(i) + "]'s last_set_command", cursor - buffer));
            }
        } catch (const std::exception& e) {
            catchToBytesError("catchToBytesError:toBytes(SrvState)", e, value, buffer_size, written_size, write_phase);
            return false;
        }
        written_size = cursor - buffer;
        return true;
    }

    bool toBytes(const CoreRequestVariantPtr& value, std::uint8_t* buffer, std::size_t buffer_size, std::size_t& written_size){
        if (!value) {
            return false;
        }
        const CoreRequestVariant& variant = *value;
        if (std::holds_alternative<SdkCommandReq>(variant)) {
            return toBytes(std::get<SdkCommandReq>(variant), buffer, buffer_size, written_size);
        } else if (std::holds_alternative<SdkConfigReq>(variant)) {
            return toBytes(std::get<SdkConfigReq>(variant), buffer, buffer_size, written_size);
        } else if (std::holds_alternative<SdkHandshakeReq>(variant)) {
            return toBytes(std::get<SdkHandshakeReq>(variant), buffer, buffer_size, written_size);
        } else if (std::holds_alternative<SdkReleaseControlReq>(variant)) {
            return toBytes(std::get<SdkReleaseControlReq>(variant), buffer, buffer_size, written_size);
        } else if (std::holds_alternative<SdkRecoveryReq>(variant)) {
            return toBytes(std::get<SdkRecoveryReq>(variant), buffer, buffer_size, written_size);
        }
        return false;
    }

    bool toBytes(const CoreResponseVariantPtr& value, std::uint8_t* buffer, std::size_t buffer_size, std::size_t& written_size){
        if (!value) {
            return false;
        }
        const CoreResponseVariant& variant = *value;
        if (std::holds_alternative<SdkCommandRes>(variant)) {
            return toBytes(std::get<SdkCommandRes>(variant), buffer, buffer_size, written_size);
        } else if (std::holds_alternative<SdkConfigRes>(variant)) {
            return toBytes(std::get<SdkConfigRes>(variant), buffer, buffer_size, written_size);
        } else if (std::holds_alternative<SdkHandshakeRes>(variant)) {
            return toBytes(std::get<SdkHandshakeRes>(variant), buffer, buffer_size, written_size);
        } else if (std::holds_alternative<SdkReleaseControlRes>(variant)) {
            return toBytes(std::get<SdkReleaseControlRes>(variant), buffer, buffer_size, written_size);
        } else if (std::holds_alternative<SdkRecoveryRes>(variant)) {
            return toBytes(std::get<SdkRecoveryRes>(variant), buffer, buffer_size, written_size);
        }
        return false;
    }

    bool toBytes(const MonitoringRequestVariantPtr& value, std::uint8_t* buffer, std::size_t buffer_size, std::size_t& written_size){
        if (!value) {
            return false;
        }
        const MonitoringRequestVariant& variant = *value;
        if (std::holds_alternative<SdkHeartbeatReq>(variant)) {
            return toBytes(std::get<SdkHeartbeatReq>(variant), buffer, buffer_size, written_size);
        } else if (std::holds_alternative<SdkSafeguardReq>(variant)) {
            return toBytes(std::get<SdkSafeguardReq>(variant), buffer, buffer_size, written_size);
        }
        return false;
    }

    bool fromBytes(const std::uint8_t* buffer, std::size_t buffer_size, SdkCommandReq& value) {
        std::vector<std::pair<std::string, std::uint8_t>> write_phase;
        const std::uint8_t* cursor = buffer;
        readRaw(cursor, value.magic_header);
        cursor += sizeof(MessageType); // Skip message type
        write_phase.push_back(std::make_pair("after message type", cursor - buffer));

        readRaw(cursor, value.client_id);
        readRaw(cursor, value.session_id);
        readRaw(cursor, value.sequence_id);
        readRaw(cursor, value.timestamp_us);
        write_phase.push_back(std::make_pair("after timestamp_us", cursor - buffer));

        readRaw(cursor, value.payload.robot_name, MAX_NAME_SIZE);
        readRaw(cursor, value.payload.arm_size);
        readRaw(cursor, value.payload.gripper_size);
        readRaw(cursor, value.payload.arm_joint_size, value.payload.arm_size);
        readRaw(cursor, value.payload.gripper_joint_size, value.payload.gripper_size);  
        write_phase.push_back(std::make_pair("after gripper_joint_size", cursor - buffer));

        readEnum(cursor, value.payload.activated_control_strategy);
        readRaw(cursor, value.payload.enable_jog);
        readRaw(cursor, value.payload.target_count);
        for (uint32_t i = 0; i < value.payload.target_count; ++i) {
            readSinglePointTarget(cursor, value.payload.target[i],
                value.payload.activated_control_strategy, value.payload.enable_jog,
                value.payload.arm_size, value.payload.arm_joint_size,
                value.payload.gripper_size, value.payload.gripper_joint_size);
        }

        return verifyReadBuffer("fromBytes(SdkCommandReq)", cursor, buffer, buffer_size, write_phase);
    }

    bool fromBytes(const std::uint8_t* buffer, std::size_t buffer_size, SdkCommandRes& value) {
        std::vector<std::pair<std::string, std::uint8_t>> write_phase;
        const std::uint8_t* cursor = buffer;
        readRaw(cursor, value.magic_header);
        cursor += sizeof(MessageType); // Skip message type
        write_phase.push_back(std::make_pair("after message type", cursor - buffer));

        readRaw(cursor, value.request_client_id);
        readRaw(cursor, value.request_sequence_id);
        readRaw(cursor, value.request_received_us);
        readRaw(cursor, value.response_sent_us);
        readEnum(cursor, value.payload.status);
        
        return verifyReadBuffer("fromBytes(SdkCommandRes)", cursor, buffer, buffer_size, write_phase);
    }

    bool fromBytes(const std::uint8_t* buffer, std::size_t buffer_size, SdkConfigReq& value) {
        std::vector<std::pair<std::string, std::uint8_t>> write_phase;
        const std::uint8_t* cursor = buffer;
        readRaw(cursor, value.magic_header);
        cursor += sizeof(MessageType); // Skip message type
        write_phase.push_back(std::make_pair("after message type", cursor - buffer));

        readRaw(cursor, value.client_id);
        readRaw(cursor, value.session_id);
        readRaw(cursor, value.sequence_id);
        readRaw(cursor, value.timestamp_us);
        write_phase.push_back(std::make_pair("after timestamp_us", cursor - buffer));

        readRaw(cursor, value.payload.robot_name, MAX_NAME_SIZE);
        readRaw(cursor, value.payload.simulation);
        readRaw(cursor, value.payload.read_only);
        readRaw(cursor, value.payload.arm_size);
        readRaw(cursor, value.payload.gripper_size);
        readRaw(cursor, value.payload.arm_joint_size, value.payload.arm_size);
        readRaw(cursor, value.payload.gripper_joint_size, value.payload.gripper_size);
        write_phase.push_back(std::make_pair("after gripper_joint_size", cursor - buffer));

        for (uint32_t i = 0; i < value.payload.arm_size; ++i) {
            readEnum(cursor, value.payload.arm[i].control_strategy);
            readEnum(cursor, value.payload.arm[i].filter_type);
            readEnum(cursor, value.payload.arm[i].target_type);
            readEnum(cursor, value.payload.arm[i].actuator_mode);
            readEnum(cursor, value.payload.arm[i].playback_cmd);
            readEnum(cursor, value.payload.arm[i].frame_reference);
            readRaw(cursor, value.payload.arm[i].reset_control_mem);
            readRaw(cursor, value.payload.arm[i].reset_interpolation);
            readRaw(cursor, value.payload.arm[i].enable_joint, value.payload.arm_joint_size[i]);
            readPositionTarget(cursor, value.payload.arm[i].tool_offset);
            readRaw(cursor, value.payload.arm[i].soft_limit_position, value.payload.arm_joint_size[i]);
            readRaw(cursor, value.payload.arm[i].soft_limit_velocity, value.payload.arm_joint_size[i]);
            readRaw(cursor, value.payload.arm[i].soft_limit_torque, value.payload.arm_joint_size[i]);
            write_phase.push_back(std::make_pair("after arm[" + std::to_string(i) + "]'s soft_limit_torque", cursor - buffer));
        }

        for (uint32_t i = 0; i < value.payload.gripper_size; ++i) {
            readRaw(cursor, value.payload.gripper[i].target_type);
            readRaw(cursor, value.payload.gripper[i].soft_limit_position, value.payload.gripper_joint_size[i]);
            write_phase.push_back(std::make_pair("after gripper[" + std::to_string(i) + "]'s soft_limit_position", cursor - buffer));
        }

        return verifyReadBuffer("fromBytes(SdkConfigReq)", cursor, buffer, buffer_size, write_phase);
    }

    bool fromBytes(const std::uint8_t* buffer, std::size_t buffer_size, SdkConfigRes& value) {
        std::vector<std::pair<std::string, std::uint8_t>> write_phase;
        const std::uint8_t* cursor = buffer;
        readRaw(cursor, value.magic_header);
        cursor += sizeof(MessageType); // Skip message type
        write_phase.push_back(std::make_pair("after message type", cursor - buffer));

        readRaw(cursor, value.request_client_id);
        readRaw(cursor, value.request_sequence_id);
        readRaw(cursor, value.request_received_us);
        readRaw(cursor, value.response_sent_us);
        write_phase.push_back(std::make_pair("after request_received_us", cursor - buffer));

        readEnum(cursor, value.payload.status);
        readRaw(cursor, value.payload.robot_name, MAX_NAME_SIZE);
        readRaw(cursor, value.payload.simulation);
        readRaw(cursor, value.payload.arm_size);
        readRaw(cursor, value.payload.gripper_size);
        readRaw(cursor, value.payload.arm_joint_size, value.payload.arm_size);
        readRaw(cursor, value.payload.gripper_joint_size, value.payload.gripper_size);
        write_phase.push_back(std::make_pair("after gripper_joint_size", cursor - buffer));

        for (uint32_t i = 0; i < value.payload.arm_size; ++i) {
            readRaw(cursor, value.payload.arm[i].name, MAX_NAME_SIZE);
            readEnum(cursor, value.payload.arm[i].control_strategy);
            readEnum(cursor, value.payload.arm[i].filter_type);
            readEnum(cursor, value.payload.arm[i].target_type);
            readEnum(cursor, value.payload.arm[i].actuator_mode);
            readEnum(cursor, value.payload.arm[i].playback_cmd);
            readEnum(cursor, value.payload.arm[i].frame_reference);
            readPositionTarget(cursor, value.payload.arm[i].tool_offset);
            write_phase.push_back(std::make_pair("after tool_offset", cursor - buffer));

            readRaw(cursor, value.payload.arm[i].enabled_joint, value.payload.arm_joint_size[i]);
            readRaw(cursor, value.payload.arm[i].soft_limit_position, value.payload.arm_joint_size[i]);
            readRaw(cursor, value.payload.arm[i].soft_limit_velocity, value.payload.arm_joint_size[i]);
            readRaw(cursor, value.payload.arm[i].soft_limit_torque, value.payload.arm_joint_size[i]);
            readRaw(cursor, value.payload.arm[i].hard_limit_position, value.payload.arm_joint_size[i]);
            readRaw(cursor, value.payload.arm[i].hard_limit_velocity, value.payload.arm_joint_size[i]);
            readRaw(cursor, value.payload.arm[i].hard_limit_torque, value.payload.arm_joint_size[i]);
            write_phase.push_back(std::make_pair("after arm[" + std::to_string(i) + "]'s hard_limit_torque", cursor - buffer));
            
            readRaw(cursor, value.payload.arm[i].follow_limit_position, value.payload.arm_joint_size[i]);
            readRaw(cursor, value.payload.arm[i].follow_limit_velocity, value.payload.arm_joint_size[i]);
            readRaw(cursor, value.payload.arm[i].follow_limit_torque, value.payload.arm_joint_size[i]);
            readRaw(cursor, value.payload.arm[i].jump_limit_position, value.payload.arm_joint_size[i]);
            readRaw(cursor, value.payload.arm[i].jump_limit_velocity, value.payload.arm_joint_size[i]);
            readRaw(cursor, value.payload.arm[i].jump_limit_torque, value.payload.arm_joint_size[i]);
            write_phase.push_back(std::make_pair("after arm[" + std::to_string(i) + "]'s jump_limit_torque", cursor - buffer));
        }

        for (uint32_t i = 0; i < value.payload.gripper_size; ++i) {
            readRaw(cursor, value.payload.gripper[i].name, MAX_NAME_SIZE);
            readEnum(cursor, value.payload.gripper[i].target_type);
            readRaw(cursor, value.payload.gripper[i].soft_limit_position, value.payload.gripper_joint_size[i]);
            readRaw(cursor, value.payload.gripper[i].hard_limit_position, value.payload.gripper_joint_size[i]);
            write_phase.push_back(std::make_pair("after gripper[" + std::to_string(i) + "]'s hard_limit_position", cursor - buffer));
        }

        return verifyReadBuffer("fromBytes(SdkConfigRes)", cursor, buffer, buffer_size, write_phase);
    }

    bool fromBytes(const std::uint8_t* buffer, std::size_t buffer_size, SdkHeartbeatReq& value) {
        std::vector<std::pair<std::string, std::uint8_t>> write_phase;
        const std::uint8_t* cursor = buffer;
        readRaw(cursor, value.magic_header);
        cursor += sizeof(MessageType); // Skip message type
        write_phase.push_back(std::make_pair("after message type", cursor - buffer));

        readRaw(cursor, value.client_id);
        readRaw(cursor, value.session_id);
        readRaw(cursor, value.timestamp_us);

        return verifyReadBuffer("fromBytes(SdkHeartbeatReq)", cursor, buffer, buffer_size, write_phase);
    }

    bool fromBytes(const std::uint8_t* buffer, std::size_t buffer_size, SdkSafeguardReq& value) {
        std::vector<std::pair<std::string, std::uint8_t>> write_phase;
        const std::uint8_t* cursor = buffer;
        readRaw(cursor, value.magic_header);
        cursor += sizeof(MessageType); // Skip message type
        write_phase.push_back(std::make_pair("after message type", cursor - buffer));

        readRaw(cursor, value.client_id);
        readRaw(cursor, value.session_id);
        readRaw(cursor, value.timestamp_us);

        return verifyReadBuffer("fromBytes(SdkSafeguardReq)", cursor, buffer, buffer_size, write_phase);
    }

    bool fromBytes(const std::uint8_t* buffer, std::size_t buffer_size, SdkHandshakeReq& value){
        std::vector<std::pair<std::string, std::uint8_t>> write_phase;
        const std::uint8_t* cursor = buffer;
        readRaw(cursor, value.magic_header);
        cursor += sizeof(MessageType); // Skip message type
        write_phase.push_back(std::make_pair("after message type", cursor - buffer));

        readRaw(cursor, value.client_id);
        readRaw(cursor, value.sequence_id);
        readRaw(cursor, value.timestamp_us);

        return verifyReadBuffer("fromBytes(SdkHandshakeReq)", cursor, buffer, buffer_size, write_phase);
    }

    bool fromBytes(const std::uint8_t* buffer, std::size_t buffer_size, SdkHandshakeRes& value){
        std::vector<std::pair<std::string, std::uint8_t>> write_phase;
        const std::uint8_t* cursor = buffer;
        readRaw(cursor, value.magic_header);
        cursor += sizeof(MessageType); // Skip message type
        write_phase.push_back(std::make_pair("after message type", cursor - buffer));

        readRaw(cursor, value.request_client_id);
        readRaw(cursor, value.request_sequence_id);
        readRaw(cursor, value.request_received_us);
        readRaw(cursor, value.response_sent_us);
        readRaw(cursor, value.assigned_session_id);

        readEnum(cursor, value.payload.status);
        std::memset(value.payload.robot_name, 0, MAX_NAME_SIZE);
        if (static_cast<std::size_t>(buffer + buffer_size - cursor) >= MAX_NAME_SIZE) {
            readRaw(cursor, value.payload.robot_name, MAX_NAME_SIZE);
        }
        return verifyReadBuffer("fromBytes(SdkHandshakeRes)", cursor, buffer, buffer_size, write_phase);
    }

    bool fromBytes(const std::uint8_t* buffer, std::size_t buffer_size, SdkReleaseControlReq& value){
        std::vector<std::pair<std::string, std::uint8_t>> write_phase;
        const std::uint8_t* cursor = buffer;
        readRaw(cursor, value.magic_header);
        cursor += sizeof(MessageType); // Skip message type
        write_phase.push_back(std::make_pair("after message type", cursor - buffer));

        readRaw(cursor, value.client_id);
        readRaw(cursor, value.session_id);
        readRaw(cursor, value.sequence_id);
        readRaw(cursor, value.timestamp_us);

        return verifyReadBuffer("fromBytes(SdkReleaseControlReq)", cursor, buffer, buffer_size, write_phase);
    }

    bool fromBytes(const std::uint8_t* buffer, std::size_t buffer_size, SdkReleaseControlRes& value){
        std::vector<std::pair<std::string, std::uint8_t>> write_phase;
        const std::uint8_t* cursor = buffer;
        readRaw(cursor, value.magic_header);
        cursor += sizeof(MessageType); // Skip message type
        write_phase.push_back(std::make_pair("after message type", cursor - buffer));

        readRaw(cursor, value.request_client_id);
        readRaw(cursor, value.request_received_us);
        readRaw(cursor, value.response_sent_us);

        readEnum(cursor, value.payload.status);
        return verifyReadBuffer("fromBytes(SdkReleaseControlRes)", cursor, buffer, buffer_size, write_phase);
    }

    bool fromBytes(const std::uint8_t* buffer, std::size_t buffer_size, SdkRecoveryReq& value){
        std::vector<std::pair<std::string, std::uint8_t>> write_phase;
        const std::uint8_t* cursor = buffer;
        readRaw(cursor, value.magic_header);
        cursor += sizeof(MessageType); // Skip message type
        write_phase.push_back(std::make_pair("after message type", cursor - buffer));

        readRaw(cursor, value.client_id);
        readRaw(cursor, value.session_id);
        readRaw(cursor, value.sequence_id);
        readRaw(cursor, value.timestamp_us);

        return verifyReadBuffer("fromBytes(SdkRecoveryReq)", cursor, buffer, buffer_size, write_phase);
    }

    bool fromBytes(const std::uint8_t* buffer, std::size_t buffer_size, SdkRecoveryRes& value){
        std::vector<std::pair<std::string, std::uint8_t>> write_phase;
        const std::uint8_t* cursor = buffer;
        readRaw(cursor, value.magic_header);
        cursor += sizeof(MessageType); // Skip message type
        write_phase.push_back(std::make_pair("after message type", cursor - buffer));

        readRaw(cursor, value.request_client_id);
        readRaw(cursor, value.request_received_us);
        readRaw(cursor, value.response_sent_us);

        readEnum(cursor, value.payload.status);
        return verifyReadBuffer("fromBytes(SdkRecoveryRes)", cursor, buffer, buffer_size, write_phase);
    }

    bool fromBytes(const std::uint8_t* buffer, std::size_t buffer_size, SrvState& value) {
        std::vector<std::pair<std::string, std::uint8_t>> write_phase;
        const std::uint8_t* cursor = buffer;
        readRaw(cursor, value.magic_header);
        cursor += sizeof(MessageType); // Skip message type
        write_phase.push_back(std::make_pair("after message type", cursor - buffer));

        readRaw(cursor, value.client_id);
        readRaw(cursor, value.sequence_id);
        readRaw(cursor, value.current_client_id);
        readRaw(cursor, value.current_client_sequence_id);
        readRaw(cursor, value.timestamp_us);
        write_phase.push_back(std::make_pair("after timestamp_us", cursor - buffer));

        readRaw(cursor, value.payload.robot_name, MAX_NAME_SIZE);
        readEnum(cursor, value.payload.system_state);
        readEnum(cursor, value.payload.plan_result);
        readRaw(cursor, value.payload.system_diagnostic_flags);
        readRaw(cursor, value.payload.session_id);
        readRaw(cursor, value.payload.last_processed_seq);
        write_phase.push_back(std::make_pair("after last_processed_seq", cursor - buffer));

        readRaw(cursor, value.payload.arm_size);
        readRaw(cursor, value.payload.gripper_size);
        readRaw(cursor, value.payload.arm_joint_size, value.payload.arm_size);
        readRaw(cursor, value.payload.gripper_joint_size, value.payload.gripper_size);
        write_phase.push_back(std::make_pair("after gripper_joint_size", cursor - buffer));

        for (uint32_t i = 0; i < value.payload.arm_size; ++i) {
            readRaw(cursor, value.payload.arm[i].position, value.payload.arm_joint_size[i]);
            readRaw(cursor, value.payload.arm[i].velocity, value.payload.arm_joint_size[i]);
            readRaw(cursor, value.payload.arm[i].torque, value.payload.arm_joint_size[i]);
            readRaw(cursor, value.payload.arm[i].acceleration, value.payload.arm_joint_size[i]);
            readCartesianArray(cursor, value.payload.arm[i].joint_poses, value.payload.arm_joint_size[i]);
            readCartesianTarget(cursor, value.payload.arm[i].tool_pose);
            readRaw(cursor, value.payload.arm[i].last_set_command, value.payload.arm_joint_size[i]);
            readRaw(cursor, value.payload.arm[i].diagnostic_flags, value.payload.arm_joint_size[i]);
            write_phase.push_back(std::make_pair("after arm[" + std::to_string(i) + "]'s diagnostic_flags", cursor - buffer));
        }

        for (uint32_t i = 0; i < value.payload.gripper_size; ++i) {
            readRaw(cursor, value.payload.gripper[i].position, value.payload.gripper_joint_size[i]);
            readRaw(cursor, value.payload.gripper[i].last_set_command, value.payload.gripper_joint_size[i]);
            readRaw(cursor, value.payload.gripper[i].diagnostic_flags, value.payload.gripper_joint_size[i]);
            write_phase.push_back(std::make_pair("after gripper[" + std::to_string(i) + "]'s diagnostic_flags", cursor - buffer));
        }

        return verifyReadBuffer("fromBytes(SrvState)", cursor, buffer, buffer_size, write_phase);
    }

    bool fromBytes(const std::uint8_t* buffer, std::size_t buffer_size, CoreRequestVariantPtr& value){
        MessageType message_type;
        if (!getReceivedMessageType(buffer, buffer_size, message_type)) {
            return false;
        }

        value = std::make_unique<CoreRequestVariant>();
        
        bool parse_success = false;
        if (message_type == MessageType::kSdkCommandReq) {
            auto& req_ref = value->emplace<SdkCommandReq>(); 
            parse_success = fromBytes(buffer, buffer_size, req_ref);
        }else if (message_type == MessageType::kSdkConfigReq) {
            auto& req_ref = value->emplace<SdkConfigReq>();
            parse_success = fromBytes(buffer, buffer_size, req_ref);
        }else if (message_type == MessageType::kSdkHandshakeReq) {
            auto& req_ref = value->emplace<SdkHandshakeReq>();
            parse_success = fromBytes(buffer, buffer_size, req_ref);
        }else if (message_type == MessageType::kSdkReleaseControlReq) {
            auto& req_ref = value->emplace<SdkReleaseControlReq>();
            parse_success = fromBytes(buffer, buffer_size, req_ref);
        }else if (message_type == MessageType::kSdkRecoveryReq) {
            auto& req_ref = value->emplace<SdkRecoveryReq>();
            parse_success = fromBytes(buffer, buffer_size, req_ref);
        }
        if (!parse_success) value = nullptr; 
        return parse_success;
    }

    bool fromBytes(const std::uint8_t* buffer, std::size_t buffer_size, CoreResponseVariantPtr& value){
        MessageType message_type;
        if (!getReceivedMessageType(buffer, buffer_size, message_type)) {
            return false;
        }

        value = std::make_shared<CoreResponseVariant>();

        bool parse_success = false;
        if (message_type == MessageType::kSdkCommandRes) {
            auto& res_ref = value->emplace<SdkCommandRes>(); 
            parse_success = fromBytes(buffer, buffer_size, res_ref);
        }else if (message_type == MessageType::kSdkConfigRes) {
            auto& res_ref = value->emplace<SdkConfigRes>();
            parse_success = fromBytes(buffer, buffer_size, res_ref);
        }else if (message_type == MessageType::kSdkHandshakeRes) {
            auto& res_ref = value->emplace<SdkHandshakeRes>();
            parse_success = fromBytes(buffer, buffer_size, res_ref);
        }else if (message_type == MessageType::kSdkReleaseControlRes) {
            auto& res_ref = value->emplace<SdkReleaseControlRes>();
            parse_success = fromBytes(buffer, buffer_size, res_ref);
        }else if (message_type == MessageType::kSdkRecoveryRes) {
            auto& res_ref = value->emplace<SdkRecoveryRes>();
            parse_success = fromBytes(buffer, buffer_size, res_ref);
        }
        if (!parse_success) value = nullptr; 
        return parse_success;
    }

    bool fromBytes(const std::uint8_t* buffer, std::size_t buffer_size, MonitoringRequestVariantPtr& value){
        MessageType message_type;
        if (!getReceivedMessageType(buffer, buffer_size, message_type)) {
            return false;
        }

        value = std::make_unique<MonitoringRequestVariant>();
        bool parse_success = false;
        if (message_type == MessageType::kSdkHeartbeatReq) {
            auto& req_ref = value->emplace<SdkHeartbeatReq>();
            parse_success = fromBytes(buffer, buffer_size, req_ref);
        }else if (message_type == MessageType::kSdkSafeguardReq) {
            auto& req_ref = value->emplace<SdkSafeguardReq>();
            parse_success = fromBytes(buffer, buffer_size, req_ref);
        }
        if (!parse_success) value = nullptr; 
        return parse_success;
    }
}  // namespace robot::platform::serialization
