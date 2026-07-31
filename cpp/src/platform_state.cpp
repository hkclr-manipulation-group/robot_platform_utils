#include "platform_state.h"
#include "time_utils.h"

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace robot::platform {
    namespace {
        constexpr std::size_t kHmacSize = 16;

        std::size_t safeBound(std::size_t requested, std::size_t max_size) {
            return std::min(requested, max_size);
        }

        std::string toSafeString(const char* value, std::size_t max_len) {
            std::size_t len = 0;
            while (len < max_len && value[len] != '\0') {
                ++len;
            }
            return std::string(value, len);
        }

        std::string toHexString(const uint8_t* data, std::size_t size) {
            std::ostringstream oss;
            oss << std::hex << std::setfill('0');
            for (std::size_t i = 0; i < size; ++i) {
                if (i > 0) {
                    oss << ' ';
                }
                oss << std::setw(2) << static_cast<int>(data[i]);
            }
            return oss.str();
        }

        template <std::size_t N>
        void printFloatArray(const std::string& label, const float (&values)[N], std::size_t count = N) {
            const std::size_t bounded = safeBound(count, N);
            std::cout << "  " << label << ": [";
            for (std::size_t i = 0; i < bounded; ++i) {
                if (i > 0) {
                    std::cout << ", ";
                }
                std::cout << values[i];
            }
            std::cout << "]\n";
        }

        template <std::size_t N, std::size_t M>
        void printFloatArray2D(const std::string& label, const float (&values)[N][M], std::size_t count = N) {
            const std::size_t bounded = safeBound(count, N);
            std::cout << "  " << label << ": [";
            for (std::size_t i = 0; i < bounded; ++i) {
                if (i > 0) {
                    std::cout << ", ";
                }
                std::cout << "[" << values[i][0] << ", " << values[i][1] << "]";
            }
            std::cout << "]\n";
        }

        template <std::size_t N>
        void printByteArray(const std::string& label, const uint8_t (&values)[N], std::size_t count = N) {
            const std::size_t bounded = safeBound(count, N);
            std::cout << "  " << label << ": [";
            for (std::size_t i = 0; i < bounded; ++i) {
                if (i > 0) {
                    std::cout << ", ";
                }
                std::cout << static_cast<int>(values[i]);
            }
            std::cout << "]\n";
        }

        void printCartesianTarget(const std::string& label, const CartesianTarget& target) {
            std::cout << "  " << label << ": "
                    << "{x: " << target.x
                    << ", y: " << target.y
                    << ", z: " << target.z
                    << ", qw: " << target.qw
                    << ", qx: " << target.qx
                    << ", qy: " << target.qy
                    << ", qz: " << target.qz
                    << "}\n";
        }

        void printPositionTarget(const std::string& label, const PositionTarget& target) {
            std::cout << "  " << label << ": "
                    << "{x: " << target.x
                    << ", y: " << target.y
                    << ", z: " << target.z
                    << "}\n";
        }
    }  // namespace

    void print(const SdkCommandReq& value) {
        std::cout << "SdkCommandReq\n";
        std::cout << "  magic_header: " << value.magic_header << "\n";
        std::cout << "  session_id: " << value.session_id << "\n";
        std::cout << "  sequence_id: " << value.sequence_id << "\n";
        std::cout << "  timestamp_us: " << value.timestamp_us << "\n";
        std::cout << "  security_hmac: " << toHexString(value.security_hmac, kHmacSize) << "\n";

        std::cout << "  payload.robot_name: " << toSafeString(value.payload.robot_name, MAX_NAME_SIZE) << "\n";
        std::cout << "  payload.arm_size: " << static_cast<int>(value.payload.arm_size) << "\n";
        std::cout << "  payload.gripper_size: " << static_cast<int>(value.payload.gripper_size) << "\n";
        printByteArray("payload.arm_joint_size", value.payload.arm_joint_size, value.payload.arm_size);
        printByteArray("payload.gripper_joint_size", value.payload.gripper_joint_size, value.payload.gripper_size);
        std::cout << "  payload.target_count: " << static_cast<int>(value.payload.target_count) << "\n";

        const std::size_t target_count = safeBound(value.payload.target_count, MAX_WAYPOINTS);
        for (std::size_t i = 0; i < target_count; ++i) {
            const auto& target = value.payload.target[i];
            std::cout << "  payload.target[" << i << "]\n";
            std::cout << "    interpolation_t: " << target.interpolation_t << "\n";
            std::cout << "    interpolation_speed_ratio: " << target.interpolation_speed_ratio << "\n";
            for (std::size_t j = 0; j < value.payload.arm_size; ++j) {
                std::string label = "arm_joint[" + std::to_string(j) + "]";
                printFloatArray(label, target.arm_joint[j], MAX_ARM_JOINT_SIZE);
            }
            for (std::size_t j = 0; j < value.payload.arm_size; ++j) {
                std::string label = "arm_tool_cartesian[" + std::to_string(j) + "]";
                printCartesianTarget(label, target.arm_tool_cartesian[j]);
            }
            for (std::size_t j = 0; j < value.payload.gripper_size; ++j) {
                std::string label = "gripper_joint[" + std::to_string(j) + "]";
                printFloatArray(label, target.gripper_joint[j], MAX_GRIPPER_JOINT_SIZE);
            }
        }
    }

    void print(const SdkCommandRes& value) {
        std::cout << "SdkCommandRes\n";
        std::cout << "  magic_header: " << value.magic_header << "\n";
        std::cout << "  request_sequence_id: " << value.request_sequence_id << "\n";
        std::cout << "  request_received_us: " << value.request_received_us << "\n";
        std::cout << "  response_sent_us: " << value.response_sent_us << "\n";
        std::cout << "  security_hmac: " << toHexString(value.security_hmac, kHmacSize) << "\n";
        std::cout << "  payload.status: " << enumToString(value.payload.status) << "\n";
    }

    void print(const SdkConfigReq& value) {
        std::cout << "SdkConfigReq\n";
        std::cout << "  magic_header: " << value.magic_header << "\n";
        std::cout << "  session_id: " << value.session_id << "\n";
        std::cout << "  sequence_id: " << value.sequence_id << "\n";
        std::cout << "  timestamp_us: " << value.timestamp_us << "\n";
        std::cout << "  security_hmac: " << toHexString(value.security_hmac, kHmacSize) << "\n";

        std::cout << "  payload.robot_name: " << toSafeString(value.payload.robot_name, MAX_NAME_SIZE) << "\n";
        std::cout << "  payload.simulation: " << static_cast<int>(value.payload.simulation) << "\n";
        std::cout << "  payload.read_only: " << static_cast<int>(value.payload.read_only) << "\n";
        std::cout << "  payload.arm_size: " << static_cast<int>(value.payload.arm_size) << "\n";
        std::cout << "  payload.gripper_size: " << static_cast<int>(value.payload.gripper_size) << "\n";
        printByteArray("payload.arm_joint_size", value.payload.arm_joint_size, value.payload.arm_size);
        printByteArray("payload.gripper_joint_size", value.payload.gripper_joint_size, value.payload.gripper_size);

        int arm_size = value.payload.arm_size;
        for (int arm_idx = 0; arm_idx < arm_size; ++arm_idx) {
            const auto& arm = value.payload.arm[arm_idx];
            int joint_size = value.payload.arm_joint_size[arm_idx];
            std::cout << "  payload.arm[" << arm_idx << "]\n";
            std::cout << "    control_strategy: " << enumToString(arm.control_strategy) << "\n";
            std::cout << "    filter_type: " << enumToString(arm.filter_type) << "\n";
            std::cout << "    target_type: " << enumToString(arm.target_type) << "\n";
            std::cout << "    actuator_mode: " << enumToString(arm.actuator_mode) << "\n";
            std::cout << "    playback_cmd: " << enumToString(arm.playback_cmd) << "\n";
            std::cout << "    frame_reference: " << enumToString(arm.frame_reference) << "\n";
            std::cout << "    reset_control_mem: " << static_cast<int>(arm.reset_control_mem) << "\n";
            std::cout << "    reset_interpolation: " << static_cast<int>(arm.reset_interpolation) << "\n";
            printByteArray("enable_joint", arm.enable_joint, joint_size);
            printPositionTarget("tool_offset", arm.tool_offset);
            printFloatArray2D("soft_limit_position", arm.soft_limit_position, joint_size);
            printFloatArray("soft_limit_velocity", arm.soft_limit_velocity, joint_size);
            printFloatArray("soft_limit_torque", arm.soft_limit_torque, joint_size);
        }

        int gripper_size = value.payload.gripper_size;
        for (int gripper_idx = 0; gripper_idx < gripper_size; ++gripper_idx) {
            const auto& gripper = value.payload.gripper[gripper_idx];
            int joint_size = value.payload.gripper_joint_size[gripper_idx];
            std::cout << "  payload.gripper[" << gripper_idx << "]\n";
            std::cout << "    target_type: " << enumToString(gripper.target_type) << "\n";
            printFloatArray2D("soft_limit_position", gripper.soft_limit_position, joint_size);
        }
    }

    void print(const SdkConfigRes& value) {
        std::cout << "SdkConfigRes\n";
        std::cout << "  magic_header: " << value.magic_header << "\n";
        std::cout << "  request_sequence_id: " << value.request_sequence_id << "\n";
        std::cout << "  request_received_us: " << value.request_received_us << "\n";
        std::cout << "  response_sent_us: " << value.response_sent_us << "\n";
        std::cout << "  security_hmac: " << toHexString(value.security_hmac, kHmacSize) << "\n";

        std::cout << "  payload.status: " << enumToString(value.payload.status) << "\n";
        std::cout << "  payload.robot_name: " << toSafeString(value.payload.robot_name, MAX_NAME_SIZE) << "\n";
        std::cout << "  payload.simulation: " << static_cast<int>(value.payload.simulation) << "\n";
        std::cout << "  payload.arm_size: " << static_cast<int>(value.payload.arm_size) << "\n";
        std::cout << "  payload.gripper_size: " << static_cast<int>(value.payload.gripper_size) << "\n";
        printByteArray("payload.arm_joint_size", value.payload.arm_joint_size, value.payload.arm_size);
        printByteArray("payload.gripper_joint_size", value.payload.gripper_joint_size, value.payload.gripper_size);

        int arm_size = value.payload.arm_size;
        for (int arm_idx = 0; arm_idx < arm_size; ++arm_idx) {
            const auto& arm = value.payload.arm[arm_idx];
            int joint_size = value.payload.arm_joint_size[arm_idx];
            std::cout << "  payload.arm[" << arm_idx << "]\n";
            std::cout << "    name: " << toSafeString(value.payload.arm[arm_idx].name, MAX_NAME_SIZE) << "\n";
            std::cout << "    control_strategy: " << enumToString(arm.control_strategy) << "\n";
            std::cout << "    filter_type: " << enumToString(arm.filter_type) << "\n";
            std::cout << "    target_type: " << enumToString(arm.target_type) << "\n";
            std::cout << "    actuator_mode: " << enumToString(arm.actuator_mode) << "\n";
            std::cout << "    frame_reference: " << enumToString(arm.frame_reference) << "\n";
            printPositionTarget("tool_offset", arm.tool_offset);
            printByteArray("enabled_joint", arm.enabled_joint, joint_size);
            printFloatArray2D("soft_limit_position", arm.soft_limit_position, joint_size);
            printFloatArray("soft_limit_velocity", arm.soft_limit_velocity, joint_size);
            printFloatArray("soft_limit_torque", arm.soft_limit_torque, joint_size);
            printFloatArray2D("hard_limit_position", arm.hard_limit_position, joint_size);
            printFloatArray("hard_limit_velocity", arm.hard_limit_velocity, joint_size);
            printFloatArray("hard_limit_torque", arm.hard_limit_torque, joint_size);
            printFloatArray("follow_limit_position", arm.follow_limit_position, joint_size);
            printFloatArray("follow_limit_velocity", arm.follow_limit_velocity, joint_size);
            printFloatArray("follow_limit_torque", arm.follow_limit_torque, joint_size);
            printFloatArray("jump_limit_position", arm.jump_limit_position, joint_size);
            printFloatArray("jump_limit_velocity", arm.jump_limit_velocity, joint_size);
            printFloatArray("jump_limit_torque", arm.jump_limit_torque, joint_size);
        }

        int gripper_size = value.payload.gripper_size;
        for (int gripper_idx = 0; gripper_idx < gripper_size; ++gripper_idx) {
            const auto& gripper = value.payload.gripper[gripper_idx];
            int joint_size = value.payload.gripper_joint_size[gripper_idx];
            std::cout << "  payload.gripper[" << gripper_idx << "]\n";
            std::cout << "    name: " << toSafeString(value.payload.gripper[gripper_idx].name, MAX_NAME_SIZE) << "\n";
            std::cout << "    target_type: " << enumToString(gripper.target_type) << "\n";
            printFloatArray2D("soft_limit_position", gripper.soft_limit_position, joint_size);
            printFloatArray2D("hard_limit_position", gripper.hard_limit_position, joint_size);
        }
    }

    void print(const SdkHandshakeReq& value){
        std::cout << "SdkHandshakeReq\n";
        std::cout << "  magic_header: " << value.magic_header << "\n";
        std::cout << "  client_id: " << value.client_id << "\n";
        std::cout << "  sequence_id: " << value.sequence_id << "\n";
        std::cout << "  timestamp_us: " << value.timestamp_us << "\n";
        std::cout << "  security_hmac: " << toHexString(value.security_hmac, kHmacSize) << "\n";
    }

    void print(const SdkHandshakeRes& value){
        std::cout << "SdkHandshakeRes\n";
        std::cout << "  magic_header: " << value.magic_header << "\n";
        std::cout << "  request_client_id: " << value.request_client_id << "\n";
        std::cout << "  request_sequence_id: " << value.request_sequence_id << "\n";
        std::cout << "  request_received_us: " << value.request_received_us << "\n";
        std::cout << "  response_sent_us: " << value.response_sent_us << "\n";
        std::cout << "  assigned_session_id: " << value.assigned_session_id << "\n";
        std::cout << "  payload.status: " << enumToString(value.payload.status) << "\n";
        std::cout << "  security_hmac: " << toHexString(value.security_hmac, kHmacSize) << "\n";
    }

    void print(const SdkReleaseControlReq& value){
        std::cout << "SdkReleaseControlReq\n";
        std::cout << "  magic_header: " << value.magic_header << "\n";
        std::cout << "  client_id: " << value.client_id << "\n";
        std::cout << "  session_id: " << value.session_id << "\n";
        std::cout << "  sequence_id: " << value.sequence_id << "\n";
        std::cout << "  timestamp_us: " << value.timestamp_us << "\n";
        std::cout << "  security_hmac: " << toHexString(value.security_hmac, kHmacSize) << "\n";
    }

    void print(const SdkReleaseControlRes& value){
        std::cout << "SdkReleaseControlRes\n";
        std::cout << "  magic_header: " << value.magic_header << "\n";
        std::cout << "  request_client_id: " << value.request_client_id << "\n";
        std::cout << "  request_sequence_id: " << value.request_sequence_id << "\n";
        std::cout << "  request_received_us: " << value.request_received_us << "\n";
        std::cout << "  response_sent_us: " << value.response_sent_us << "\n";
        std::cout << "  payload.status: " << enumToString(value.payload.status) << "\n";
        std::cout << "  security_hmac: " << toHexString(value.security_hmac, kHmacSize) << "\n";
    }

    void print(const SdkRecoveryReq& value){
        std::cout << "SdkRecoveryReq\n";
        std::cout << "  magic_header: " << value.magic_header << "\n";
        std::cout << "  client_id: " << value.client_id << "\n";
        std::cout << "  session_id: " << value.session_id << "\n";
        std::cout << "  sequence_id: " << value.sequence_id << "\n";
        std::cout << "  timestamp_us: " << value.timestamp_us << "\n";
        std::cout << "  security_hmac: " << toHexString(value.security_hmac, kHmacSize) << "\n";
    }

    void print(const SdkRecoveryRes& value){
        std::cout << "SdkRecoveryRes\n";
        std::cout << "  magic_header: " << value.magic_header << "\n";
        std::cout << "  request_client_id: " << value.request_client_id << "\n";
        std::cout << "  request_sequence_id: " << value.request_sequence_id << "\n";
        std::cout << "  request_received_us: " << value.request_received_us << "\n";
        std::cout << "  response_sent_us: " << value.response_sent_us << "\n";
        std::cout << "  payload.status: " << enumToString(value.payload.status) << "\n";
        std::cout << "  security_hmac: " << toHexString(value.security_hmac, kHmacSize) << "\n";
    }

    void print(const SdkHeartbeatReq& value) {
        std::cout << "SdkHeartbeatReq\n";
        std::cout << "  magic_header: " << value.magic_header << "\n";
        std::cout << "  session_id: " << value.session_id << "\n";
        std::cout << "  timestamp_us: " << value.timestamp_us << "\n";
        std::cout << "  security_hmac: " << toHexString(value.security_hmac, kHmacSize) << "\n";
    }

    void print(const SdkSafeguardReq& value) {
        std::cout << "SdkSafeguardReq\n";
        std::cout << "  magic_header: " << value.magic_header << "\n";
        std::cout << "  session_id: " << value.session_id << "\n";
        std::cout << "  timestamp_us: " << value.timestamp_us << "\n";
        std::cout << "  security_hmac: " << toHexString(value.security_hmac, kHmacSize) << "\n";
    }

    void print(const SrvState& value) {
        std::cout << "SrvState\n";
        std::cout << "  magic_header: " << value.magic_header << "\n";
        std::cout << "  sequence_id: " << value.sequence_id << "\n";
        std::cout << "  current_client_id: " << value.current_client_id << "\n";
        std::cout << "  current_client_sequence_id: " << value.current_client_sequence_id << "\n";
        std::cout << "  timestamp_us: " << value.timestamp_us << "\n";
        std::cout << "  security_hmac: " << toHexString(value.security_hmac, kHmacSize) << "\n";

        std::cout << "  payload.robot_name: " << toSafeString(value.payload.robot_name, MAX_NAME_SIZE) << "\n";
        std::cout << "  payload.system_state: " << enumToString(value.payload.system_state) << "\n";
        std::cout << "  payload.plan_result: " << enumToString(value.payload.plan_result) << "\n";
        std::cout << "  payload.system_diagnostic_flags: \n";
        print_diagnostic_flags(value.payload.system_diagnostic_flags);
        std::cout << "  payload.session_id: " << value.payload.session_id << "\n";
        std::cout << "  payload.last_processed_seq: " << value.payload.last_processed_seq << "\n";

        int arm_size = value.payload.arm_size;
        for (std::size_t arm_idx = 0; arm_idx < arm_size; ++arm_idx) {
            const auto& arm = value.payload.arm[arm_idx];
            int joint_size = value.payload.arm_joint_size[arm_idx];
            std::cout << "  payload.arm[" << arm_idx << "]\n";
            printFloatArray("position", arm.position, joint_size);
            printFloatArray("velocity", arm.velocity, joint_size);
            printFloatArray("torque", arm.torque, joint_size);
            printFloatArray("acceleration", arm.acceleration, joint_size);
            for (std::size_t joint_idx = 0; joint_idx < static_cast<std::size_t>(joint_size); ++joint_idx) {
                printCartesianTarget("joint_poses[" + std::to_string(joint_idx) + "]", arm.joint_poses[joint_idx]);
            }
            printCartesianTarget("tool_pose", arm.tool_pose);
            printFloatArray("last_set_command", arm.last_set_command, joint_size);
            for (std::size_t joint_idx = 0; joint_idx < static_cast<std::size_t>(joint_size); ++joint_idx) {
                std::cout << "diagnostic_flags[" << joint_idx << "]: ";
                print_diagnostic_flags(arm.diagnostic_flags[joint_idx]);
            }
        }

        int gripper_size = value.payload.gripper_size;
        for (int gripper_idx = 0; gripper_idx < gripper_size; ++gripper_idx) {
            const auto& gripper = value.payload.gripper[gripper_idx];
            int joint_size = value.payload.gripper_joint_size[gripper_idx];
            std::cout << "  payload.gripper[" << gripper_idx << "]\n";
            printFloatArray("position", gripper.position, joint_size);
            printFloatArray("last_set_command", gripper.last_set_command, joint_size);
            for (std::size_t joint_idx = 0; joint_idx < static_cast<std::size_t>(joint_size); ++joint_idx) {
                std::cout << "diagnostic_flags[" << joint_idx << "]: ";
                print_diagnostic_flags(gripper.diagnostic_flags[joint_idx]);
            }
        }
    }

    void print_diagnostic_flags(uint64_t flags){
        if (flags == DiagnosticFlags::kNone){
            printf("None\n");
            return;
        }
        
        if (flags & DiagnosticFlags::kTargetPosSaturation){
            printf("TargetPosSaturation: User target clamped by soft position limits\n");
        }
        if (flags & DiagnosticFlags::kTargetVelSaturation){
            printf("TargetVelSaturation: User target clamped by soft velocity limits\n");
        }
        if (flags & DiagnosticFlags::kTargetTorSaturation){
            printf("TargetTorSaturation: User target clamped by soft torque limits\n");
        }
        if (flags & DiagnosticFlags::kActuatorPosSaturation){
            printf("ActuatorPosSaturation: Actuator command clamped by soft position limits\n");
        }
        if (flags & DiagnosticFlags::kActuatorVelSaturation){
            printf("ActuatorVelSaturation: Actuator command clamped by soft velocity limits\n");
        }
        if (flags & DiagnosticFlags::kActuatorTorSaturation){
            printf("ActuatorTorSaturation: Actuator command clamped by soft torque limits\n");
        }
        if (flags & DiagnosticFlags::kActuatorPosJumpSaturation){
            printf("ActuatorPosJumpSaturation: Actuator command clamped by position jump limit\n");
        }
        if (flags & DiagnosticFlags::kActuatorVelJumpSaturation){
            printf("ActuatorVelJumpSaturation: Actuator command clamped by velocity jump limit\n");
        }
        if (flags & DiagnosticFlags::kActuatorTorJumpSaturation){
            printf("ActuatorTorJumpSaturation: Actuator command clamped by torque jump limit\n");
        }
        if (flags & DiagnosticFlags::kBoundaryVelClamp){
            printf("BoundaryVelClamp: Velocity zeroed in limit direction due to position boundary reached\n");
        }
        if (flags & DiagnosticFlags::kBoundaryJointImpedance){
            printf("BoundaryJointImpedance: Joint impedance applied due to position boundary reached\n");
        }

        // --- Algorithmic & Kinematic Planner Modifications ---
        if (flags & DiagnosticFlags::kPlanTimelineExtended){
            printf("PlanTimelineExtended: Trajectory segment duration (dt) stretched for velocity limits\n");
        }
        if (flags & DiagnosticFlags::kPlanVelLimitInvalid){
            printf("PlanVelLimitInvalid: Specified max velocity profile is below tolerance(1e-4)\n");
        }
        if (flags & DiagnosticFlags::kPlanDeltaTooLarge){
            printf("PlanDeltaTooLarge: Distance between points requires unachievable time scaling\n");
        }
        if (flags & DiagnosticFlags::kPlanVelocitySnap){
            printf("PlanVelocitySnap: Large velocity shift over zero distance\n");
        }
        if (flags & DiagnosticFlags::kPlanPointSkipped){
            printf("PlanPointSkipped: Duplicated points skipped\n");
        }

        // --- Fault Flags ---
        if (flags & DiagnosticFlags::kFaultPosHardLimitReached){
            printf("FaultPosHardLimitReached: Position hard limit reached\n");
        }
        if (flags & DiagnosticFlags::kFaultVelHardLimitReached){
            printf("FaultVelHardLimitReached: Velocity hard limit reached\n");
        }
        if (flags & DiagnosticFlags::kFaultTorHardLimitReached){
            printf("FaultTorHardLimitReached: Torque hard limit reached\n");
        }
        if (flags & DiagnosticFlags::kFaultPosTrackingFailed){
            printf("FaultPosTrackingFailed: Position tracking failed\n");
        }
        if (flags & DiagnosticFlags::kFaultVelTrackingFailed){
            printf("FaultVelTrackingFailed: Velocity tracking failed\n");
        }
        if (flags & DiagnosticFlags::kFaultTorTrackingFailed){
            printf("FaultTorTrackingFailed: Torque tracking failed\n");
        }
        if (flags & DiagnosticFlags::kFaultArmNotFound){
            printf("FaultArmNotFound: Arm not found\n");
        }
        if (flags & DiagnosticFlags::kFaultGripperNotFound){
            printf("FaultGripperNotFound: Gripper not found\n");
        }
        if (flags & DiagnosticFlags::kFaultButtonNotFound){
            printf("kFaultButtonNotFound: Button not found\n");
        }
        if (flags & DiagnosticFlags::kFaultArmInitFailed){
            printf("kFaultArmInitFailed: Arm Hardware initialization failed\n");
        }
        if (flags & DiagnosticFlags::kFaultGripperInitFailed){
            printf("kFaultGripperInitFailed: Gripper Hardware initialization failed\n");
        }
        if (flags & DiagnosticFlags::kFaultButtonInitFailed){
            printf("kFaultButtonInitFailed: Button Hardware initialization failed\n");
        }
        if (flags & DiagnosticFlags::kFaultHardwareEnableFailed){
            printf("kFaultHardwareEnableFailed: Hardware failed to enable\n");
        }
        if (flags & DiagnosticFlags::kFaultHardwareChangeModeFailed){
            printf("kFaultHardwareChangeModeFailed: Hardware failed to change mode\n");
        }
        if (flags & DiagnosticFlags::kFaultHardwareSetReadFailed){
            printf("kFaultHardwareSetReadFailed: Hardware failed to Set or Read\n");
        }

        if (flags & DiagnosticFlags::kFaultArmSizeMismatch){
            printf("FaultArmSizeMismatch: SDK arm count does not match server configuration\n");
        }
        if (flags & DiagnosticFlags::kFaultGripperSizeMismatch){
            printf("FaultGripperSizeMismatch: SDK gripper count does not match server configuration\n");
        }
        if (flags & DiagnosticFlags::kFaultArmJointSizeMismatch){
            printf("FaultArmJointSizeMismatch: SDK arm joint count does not match server configuration\n");
        }
        if (flags & DiagnosticFlags::kFaultGripperJointSizeMismatch){
            printf("FaultGripperJointSizeMismatch: SDK gripper joint count does not match server configuration\n");
        }
        if (flags & DiagnosticFlags::kFaultRobotNameMismatch){
            printf("FaultRobotNameMismatch: SDK robot name does not match server configuration\n");
        }
        if (flags & DiagnosticFlags::kFaultCollisionDetected){
            printf("FaultCollisionDetected: Collision detected\n");
        }
        if (flags & DiagnosticFlags::kFaultRecoveryRequired){
            printf("FaultRecoveryRequired: Recovery required\n");
        }
        if (flags & DiagnosticFlags::kInvalidControlTypeCombination){
            printf("InvalidControlTypeCombination: Control type combination is invalid or not supported\n");
        }
        if (flags & DiagnosticFlags::kCartesianControlRequirePositionTarget){
            printf("CartesianControlRequirePositionTarget: Cartesian control require position target\n");
        }
        if (flags & DiagnosticFlags::kControlStrategyNotAvailable){
            printf("ControlStrategyNotAvailable: Control strategy is not available\n");
        }
        if (flags & DiagnosticFlags::kWaypointControlStrategyNotAllowed){
            printf("WaypointControlStrategyNotAllowed: Control strategy is not allowed for waypoint\n");
        }
        if (flags & DiagnosticFlags::kWaypointSmoothingMethodNotAllowed){
            printf("WaypointSmoothingMethodNotAllowed: Smoothing method is not allowed for waypoint\n");
        }
        if (flags & DiagnosticFlags::kWaypointTargetTypeNotAllowed){
            printf("WaypointTargetTypeNotAllowed: Target type is not allowed for waypoint\n");
        }
        if (flags & DiagnosticFlags::kPlaybackControlRequirePositionTarget){
            printf("PlaybackControlRequirePositionTarget: Playback control require position target\n");
        }
        if (flags & DiagnosticFlags::kPlaybackControlStartPoseNotReachable){
            printf("PlaybackControlStartPoseNotReachable: Playback control start pose not reachable\n");
        }
        if (flags & DiagnosticFlags::kUnknown){
            printf("FaultUnknown: Unknown fault\n");
        }
    }
    
    std::pair<std::string, std::string> getStatusErrorMessage(CommandResponseStatus status){
        std::string title;
        std::string message;

        switch (status) {
            case CommandResponseStatus::kSuccess:
                title = "Success";
                message = "Command executed successfully.";
                break;

            case CommandResponseStatus::kRejectedBlocked:
                title = "Access Denied";
                message = "Another client with higher priority is currently controlling the robot.\n"
                        "Please wait for the other client to release control, or try again.";
                break;

            case CommandResponseStatus::kRejectedConfigNotReady:
                title = "System Busy";
                message = "A configuration update is currently in progress.\n"
                        "The requested configuration is not yet active. Please wait.";
                break;

            case CommandResponseStatus::kRejectedFaulted:
                title = "System Fault";
                message = "The robot is currently in a faulted state.\n"
                        "A recovery operation is required to clear the fault.";
                break;

            case CommandResponseStatus::kRejectedSequenceId:
                title = "Sequence Error";
                message = "The command sequence ID is outdated.\n"
                        "The system will resynchronize and retry automatically.";
                break;

            case CommandResponseStatus::kRejectedConfigRequired:
                title = "Configuration Missing";
                message = "Initial configuration is missing.\n"
                        "You must set the robot configuration at least once.";
                break;

            case CommandResponseStatus::kInvalidCommand:
                title = "Validation Failed";
                message = "Packet payload failed structural or cryptographic validation.";
                break;

            case CommandResponseStatus::kInvalidClientId:
            case CommandResponseStatus::kInvalidSessionId:
                title = "Authentication Failed";
                message = "Invalid Client ID or Session ID.\n"
                        "The panel might have disconnected. Please re-authenticate.";
                break;

            case CommandResponseStatus::kInvalidConfig:
                title = "Invalid Configuration";
                message = "The sent configuration parameters are invalid.\n"
                        "Please verify your settings and try again.";
                break;

            case CommandResponseStatus::kTimeout:
                title = "Timeout Error";
                message = "The configuration update timed out.\n"
                        "Please check the network connection and try again.";
                break;

            default:
                title = "Unknown Error";
                message = "An unknown error occurred (Error Code: " + std::to_string(static_cast<int>(status)) + ").";
                break;
            }
        return {title, message};
    }

    SdkConfigReq getReadOnlyConfigRequest(uint16_t client_id, uint32_t session_id, uint32_t sequence_id){
        SdkConfigReq req;
        req.magic_header = MAGIC_HEADER;
        req.client_id = client_id;
        req.session_id = session_id;
        req.sequence_id = sequence_id;
        req.timestamp_us = get_time_now();
        req.payload.read_only = uint8_t(true);
        req.payload.arm_size = 0;
        req.payload.gripper_size = 0;
        return req;
    }

    SdkCommandReq getInitialCommandRequest(uint16_t client_id, uint32_t session_id, uint32_t sequence_id){
        SdkCommandReq req{};
        req.magic_header = MAGIC_HEADER;
        req.client_id = client_id;
        req.session_id = session_id;
        req.sequence_id = sequence_id;
        req.timestamp_us = get_time_now();
        req.payload.target_count = 0;
        return req;
    }
    
    void updateConfigRequest(const SdkConfigRes& response, SdkConfigReq& request){
        request.magic_header = MAGIC_HEADER;
        request.client_id = response.request_client_id;
        request.sequence_id = response.request_sequence_id;
        request.timestamp_us = get_time_now();

        std::snprintf(request.payload.robot_name, MAX_NAME_SIZE, "%s", response.payload.robot_name);
        request.payload.simulation = response.payload.simulation;

        request.payload.arm_size = response.payload.arm_size;
        request.payload.gripper_size = response.payload.gripper_size;
        for (int arm_i = 0; arm_i < response.payload.arm_size; ++arm_i){
            request.payload.arm_joint_size[arm_i] = response.payload.arm_joint_size[arm_i];
        }
        for (int gripper_i = 0; gripper_i < response.payload.gripper_size; ++gripper_i){
            request.payload.gripper_joint_size[gripper_i] = response.payload.gripper_joint_size[gripper_i];
        }

        for (int arm_i = 0; arm_i < response.payload.arm_size; ++arm_i){
            request.payload.arm[arm_i].control_strategy = response.payload.arm[arm_i].control_strategy;
            request.payload.arm[arm_i].filter_type = response.payload.arm[arm_i].filter_type;
            request.payload.arm[arm_i].target_type = response.payload.arm[arm_i].target_type;
            request.payload.arm[arm_i].actuator_mode = response.payload.arm[arm_i].actuator_mode;
            request.payload.arm[arm_i].frame_reference = response.payload.arm[arm_i].frame_reference;
            request.payload.arm[arm_i].tool_offset = response.payload.arm[arm_i].tool_offset;

            for (int joint_i = 0; joint_i < response.payload.arm_joint_size[arm_i]; ++joint_i){
                request.payload.arm[arm_i].enable_joint[joint_i] = response.payload.arm[arm_i].enabled_joint[joint_i];
                request.payload.arm[arm_i].soft_limit_position[joint_i][0] = response.payload.arm[arm_i].soft_limit_position[joint_i][0];
                request.payload.arm[arm_i].soft_limit_position[joint_i][1] = response.payload.arm[arm_i].soft_limit_position[joint_i][1];
                request.payload.arm[arm_i].soft_limit_velocity[joint_i] = response.payload.arm[arm_i].soft_limit_velocity[joint_i];
                request.payload.arm[arm_i].soft_limit_torque[joint_i] = response.payload.arm[arm_i].soft_limit_torque[joint_i];
            }
        }

        for (int gripper_i = 0; gripper_i < response.payload.gripper_size; ++gripper_i){
            request.payload.gripper[gripper_i].target_type = response.payload.gripper[gripper_i].target_type;

            for (int joint_i = 0; joint_i < response.payload.gripper_joint_size[gripper_i]; ++joint_i){
                request.payload.gripper[gripper_i].soft_limit_position[joint_i][0] = response.payload.gripper[gripper_i].soft_limit_position[joint_i][0];
                request.payload.gripper[gripper_i].soft_limit_position[joint_i][1] = response.payload.gripper[gripper_i].soft_limit_position[joint_i][1];
            }
        }
    }
}  // namespace robot::platform