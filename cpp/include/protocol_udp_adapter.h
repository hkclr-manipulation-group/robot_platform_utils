#pragma once
#include "protocol_udp.h"
#include "protocol_v2_service.h"
#include "platform_state.h"

namespace robot::platform::protocol_udp {
// Hash semantic configuration fields only, never native struct padding or request metadata.
inline std::uint64_t configurationRevision(const SdkConfigReq& config) {
    base::Writer bytes;
    const auto& p = config.payload;
    if (!p.arm_size || p.arm_size > MAX_ARM_SIZE || p.gripper_size > MAX_GRIPPER_SIZE) return 0;
    bytes.put(p.arm_size); bytes.put(p.gripper_size);
    for (int a = 0; a < p.arm_size; ++a) {
        const auto count = p.arm_joint_size[a];
        if (!count || count > MAX_ARM_JOINT_SIZE) return 0;
        bytes.put(count);
        const auto& arm = p.arm[a];
        bytes.put(static_cast<std::uint8_t>(arm.control_strategy)); bytes.put(static_cast<std::uint8_t>(arm.target_type));
        bytes.put(static_cast<std::uint8_t>(arm.actuator_mode)); bytes.put(static_cast<std::uint8_t>(arm.filter_type));
        for (int j = 0; j < count; ++j) {
            bytes.put(static_cast<std::uint8_t>(arm.enable_joint[j]));
            putFloat(bytes, arm.soft_limit_position[j][0]); putFloat(bytes, arm.soft_limit_position[j][1]);
            putFloat(bytes, arm.soft_limit_velocity[j]); putFloat(bytes, arm.soft_limit_torque[j]);
        }
    }
    for (int g = 0; g < p.gripper_size; ++g) {
        const auto count = p.gripper_joint_size[g];
        if (!count || count > MAX_GRIPPER_JOINT_SIZE) return 0;
        bytes.put(count);
        bytes.put(static_cast<std::uint8_t>(p.gripper[g].target_type));
        for (int j = 0; j < count; ++j) {
            putFloat(bytes, p.gripper[g].soft_limit_position[j][0]);
            putFloat(bytes, p.gripper[g].soft_limit_position[j][1]);
        }
    }
    std::uint64_t hash = 14695981039346656037ull;
    for (auto b : bytes.data) hash = (hash ^ b) * 1099511628211ull;
    return hash ? hash : 1;
}
// Validate the entire atomic arm group before touching output. No gripper motion,
// mixed modes or omitted arms in this first motion schema.
inline bool adapt(const Packet& packet, const SdkConfigReq& config, const SrvState& state, SdkCommandReq& out) {
    if (!valid(packet.command) || packet.command.targets.size() != config.payload.arm_size ||
        !configurationRevision(config)) return false;
    SdkCommandReq command{};
    command.internal_v2_command_id = packet.context.command;
    command.client_id = packet.context.client; command.session_id = packet.context.session;
    command.sequence_id = packet.context.sequence; command.timestamp_us = packet.context.issued_us;
    std::memcpy(command.payload.robot_name, config.payload.robot_name, MAX_NAME_SIZE);
    command.payload.arm_size = config.payload.arm_size; command.payload.gripper_size = config.payload.gripper_size;
    command.payload.target_count = 1;
    auto& target = command.payload.target[0];
    target.interpolation_t = packet.command.duration_sec;
    target.interpolation_speed_ratio = packet.command.velocity_scale;
    for (int a = 0; a < config.payload.arm_size; ++a) {
        const auto& arm = config.payload.arm[a];
        if (arm.control_strategy != ControlStrategy::kJoint || arm.target_type != ControlType::kPosition ||
            (arm.actuator_mode != ControlType::kPosition && arm.actuator_mode != ControlType::kVelocity) ||
            (arm.filter_type != SmoothingMethod::kCos && arm.filter_type != SmoothingMethod::kLinear &&
             arm.filter_type != SmoothingMethod::kCubic && arm.filter_type != SmoothingMethod::kQuintic)) return false;
        const Target* selected = nullptr;
        for (const auto& t : packet.command.targets) if (t.component == std::uint32_t(a + 1)) selected = &t;
        const auto count = config.payload.arm_joint_size[a];
        if (!selected || selected->radians.size() != count) return false;
        command.payload.arm_joint_size[a] = count;
        command.payload.activated_control_strategy[a] = ControlStrategy::kJoint;
        for (int j = 0; j < count; ++j) {
            const auto q = selected->radians[j];
            const auto lo = arm.soft_limit_position[j][0], hi = arm.soft_limit_position[j][1];
            if (!std::isfinite(lo) || !std::isfinite(hi) || lo > hi || q < lo || q > hi) return false;
            target.arm_joint[a][j] = q;
        }
    }
    for (int g = 0; g < config.payload.gripper_size; ++g) {
        if (config.payload.gripper[g].target_type != ControlType::kPosition) return false;
        const auto count = config.payload.gripper_joint_size[g];
        if (count > MAX_GRIPPER_JOINT_SIZE || state.payload.gripper_size <= g || state.payload.gripper_joint_size[g] != count) return false;
        command.payload.gripper_joint_size[g] = count;
        for (int j = 0; j < count; ++j) {
            const auto q = state.payload.gripper[g].position[j];
            if (!std::isfinite(q)) return false;
            target.gripper_joint[g][j] = q;
        }
    }
    out = command; return true;
}
} // namespace robot::platform::protocol_udp
