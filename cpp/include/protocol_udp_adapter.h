#pragma once
#include "protocol_udp.h"
#include "protocol_v2_service.h"
#include "platform_state.h"
#include <algorithm>

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
    // Preserve the existing Joint-only revision. Cartesian commands additionally
    // bind the configured TCP; changing it must invalidate prepared operations.
    if (std::any_of(p.arm,p.arm+p.arm_size,[](const auto& a) {
            return a.control_strategy==ControlStrategy::kCartesian || a.control_strategy==ControlStrategy::kCartesianLine; })) {
        bytes.put(std::uint32_t(0x43545031));
        for (int a=0;a<p.arm_size;++a) {
            putFloat(bytes,p.arm[a].tool_offset.x); putFloat(bytes,p.arm[a].tool_offset.y); putFloat(bytes,p.arm[a].tool_offset.z);
        }
    }
    std::uint64_t hash = 14695981039346656037ull;
    for (auto b : bytes.data) hash = (hash ^ b) * 1099511628211ull;
    return hash ? hash : 1;
}
// Validate the entire atomic arm group before touching output. No gripper motion,
// omitted arms; typed targets keep mixed planning spaces out of the Legacy union.
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
        if ((arm.control_strategy != ControlStrategy::kJoint && arm.control_strategy != ControlStrategy::kCartesian &&
             arm.control_strategy != ControlStrategy::kCartesianLine) || arm.target_type != ControlType::kPosition ||
            (arm.actuator_mode != ControlType::kPosition && arm.actuator_mode != ControlType::kVelocity) ||
            (arm.filter_type != SmoothingMethod::kCos && arm.filter_type != SmoothingMethod::kLinear &&
             arm.filter_type != SmoothingMethod::kCubic && arm.filter_type != SmoothingMethod::kQuintic)) return false;
        const Target* selected = nullptr;
        for (const auto& t : packet.command.targets) if (t.component == std::uint32_t(a + 1)) selected = &t;
        const auto count = config.payload.arm_joint_size[a];
        const auto expected=arm.control_strategy==ControlStrategy::kJoint ? 1u : arm.control_strategy==ControlStrategy::kCartesian ? 2u : 3u;
        if (!selected || selected->type!=expected || (expected==1 && selected->radians.size()!=count)) return false;
        // Mixed-space first release uses a common physical Position mode.
        const bool mixed=std::any_of(config.payload.arm,config.payload.arm+config.payload.arm_size,
                [](const auto& x) { return x.control_strategy!=ControlStrategy::kJoint; });
        if (mixed && config.payload.arm_size<2) return false;
        if (mixed && arm.actuator_mode!=ControlType::kPosition) return false;
        command.payload.arm_joint_size[a] = count;
        command.payload.activated_control_strategy[a] = arm.control_strategy;
        auto& typed=target.typed_arm[a];
        if (mixed) {
            typed.kind=static_cast<ArmTargetKind>(expected); typed.joint_count=count;
        }
        if (expected!=1) {
            typed.pose={selected->pose[0],selected->pose[1],selected->pose[2],selected->pose[3],
                selected->pose[4],selected->pose[5],selected->pose[6]};
            continue;
        }
        for (int j = 0; j < count; ++j) {
            const auto q = selected->radians[j];
            const auto lo = arm.soft_limit_position[j][0], hi = arm.soft_limit_position[j][1];
            if (!std::isfinite(lo) || !std::isfinite(hi) || lo > hi || q < lo || q > hi) return false;
            if (mixed) typed.joints[j]=q;
            else target.arm_joint[a][j] = q;
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
