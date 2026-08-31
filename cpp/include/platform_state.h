#ifndef PLATFORM_STATE_H
#define PLATFORM_STATE_H

#include <inttypes.h>
#include <string>
#include <variant>
#include <vector>
#include <memory>
#include <cstddef>
#include <type_traits>
#include "3rd_party/magic_enum.hpp"

namespace robot::platform {
    #define MAX_ARM_SIZE            2
    #define MAX_ARM_JOINT_SIZE      7
    #define MAX_GRIPPER_SIZE        MAX_ARM_SIZE
    #define MAX_GRIPPER_JOINT_SIZE  6
    #define MAX_NAME_SIZE           30
    #define MAX_WAYPOINTS           32
    #define MAX_NETWORK_IP_SIZE     16
    #define MAX_NETWORK_DNS_SIZE    64
    #define MAX_WIFI_SSID_SIZE      64
    #define MAX_WIFI_PASSWORD_SIZE  64
    #define MAGIC_HEADER            0x484B434C // "HKCL" in ASCII
    #ifndef HMAC_KEY_SIZE
        #define HMAC_KEY_SIZE           16U
    #endif

    /**
     * @brief The control strategy to apply.
     * @param kJoint The joint space control strategy.
     * @param kCartesian The cartesian space control strategy.
     * @param kCartesianLine The cartesian line control strategy.
     * @param kNullspace The nullspace control strategy.
     * @param kGravityCompensation The gravity compensation control strategy.
     * @param kRecord The record control strategy.
     * @param kPlayback The playback control strategy.
     */
    enum class ControlStrategy: uint8_t{
        kJoint = 0, 
        kCartesian, 
        kCartesianLine, 
        kNullspace, 
        kGravityCompensation,
        kTeachingReplay,
    };

    enum class FrameReference: uint8_t { kTool, kBase };

    /**
     * @brief The smoothing method to apply.
     * @param kLinear The linear smoothing method.
     * @param kCos The cosine smoothing method.
     * @param kCubic The cubic smoothing method.
     * @param kQuintic The quintic smoothing method.
     * @param kQuinticPath The quintic path smoothing method.
     * @param kLowPassFilter The low pass filter smoothing method.
     * @param kNone The none smoothing method.
     */
    enum class SmoothingMethod: uint8_t{
        kLinear,
        kCos,
        kCubic,
        kQuintic,
        kQuinticPath,
        kLowPassFilter,
        kNone,
    };

    /**
     * @brief The motion phase to apply.
     * @param kIdle The idle motion phase.
     * @param kInitialized The initialized motion phase.
     * @param kAcceleration The acceleration motion phase.
     * @param kConstantVelocity The constant velocity motion phase.
     * @param kDeceleration The deceleration motion phase.
     * @param kFiltering The filtering motion phase.
     * @param kFinished The finished motion phase.
     * @param kInterrupted The interrupted motion phase.
     */
    enum class MotionPhase : uint8_t {
        kIdle,
        kInitialized,
        kAcceleration,
        kConstantVelocity,
        kDeceleration,
        kFiltering,
        kFinished,
        kInterrupted,
    };

    /**
     * @brief The control type to apply.
     * @param kPosition The position control type.
     * @param kVelocity The velocity control type.
     * @param kTorque The torque control type.
     * @param kInvalid The invalid control type.
     */
    enum class ControlType: uint8_t{
        kPosition=0, 
        kVelocity, 
        kTorque,
        kInvalid=255,
    };

    /**
     * @brief The cartesian line move strategy to apply.
     * @param kRejectEntirely The reject entirely cartesian line move strategy.
     * @param kStopAtBoundary The stop at boundary cartesian line move strategy.
     * @param kDeviateAndBypassing The deviate and bypassing cartesian line move strategy.
     * @param kSegmentedExecution The segmented execution cartesian line move strategy.
     */
    enum class CartesianLineMoveStrategy : uint8_t  {
        kRejectEntirely = 0, // Abort the move immediately; do not start moving.
        kStopAtBoundary = 1, // Move and stop smoothly at the last reachable point before the failure.
        kDeviateAndBypassing = 2, // Deviate from the line (e.g., switch to joint space) to bypass the unreachable zone.
        kSegmentedExecution = 3 // Execute valid parts, skip the bad segment, and resume line tracking later.
    };
    
    /**
    * @brief State Machine Constraints for Robot Teaching Features
    * 
    * 1. Idle/Default State (`kNone`):
    *    - Constraint: None. Safe to call in any ControlStrategy.
    *    - Note: Performs the same safe-stop operation as `kStopRecord` and `kStopReplay`.
    * 
    * 2. Recording Commands (`kStartRecord`, `kStopRecord`):
    *    - Constraint: Requires `ControlStrategy != ControlStrategy::kTeachingReplay`
    * 
    * 3. Replay Commands (`kStartReplay`, `kStopReplay`, `kResetReplay`):
    *    - Constraint: Requires `ControlStrategy == ControlStrategy::kTeachingReplay`
    *    - Note: Silently ignored if received under any non-replay ControlStrategy.
    */
    enum class TeachingCommand: uint8_t{kNone, kStartRecord, kStopRecord, kStartReplay, kStopReplay, kResetReplay};

    /**
     * @brief The playback state to apply.
     * @param kStop The stop playback state.
     * @param kStart The start playback state.
     * @param kReset The reset playback state.
     */
    enum class PlaybackState: uint8_t{
        kStop = 0,
        kStart = 1,
        kReset = 2,
    };

    /** Per-axis jog request when SdkCommandReq.payload.enable_jog is set. */
    /**
     * @brief The jog command to apply.
     * @param kStop The stop jog command.
     * @param kIncrease The increase jog command.
     * @param kDecrease The decrease jog command.
     */
    enum class JogCommand: uint8_t {
        kStop = 0,
        kIncrease = 1,
        kDecrease = 2,
    };
    
    /**
     * @brief The plan result to apply.
     * @param kSuccess The success plan result.
     * @param kPoseNotReachable The pose not reachable plan result.
     * @param kLinearPathFailed The linear path failed plan result.
     */
    enum class PlanResult : uint8_t {
        kNone                       = 0, //No applicable
        kSuccess                    = 1,  // IK passed, time allocation valid, safe to execute
        kPoseNotReachable           = 2,  // Target 6D pose is physically outside the workspace
        kLinearPathFailed           = 3,  // Reachable target, but continuous linear path is blocked (Singularity / Joint Limit)
    };

    /**
     * @brief The system state to apply.
     * @param kUnknown The unknown system state.
     * @param kStartup The startup system state.
     * @param kIdle The idle system state.
     * @param kMoving The moving system state.
     * @param kSettling The settling system state.
     * @param kError The error system state.
     * @param kRecovery The recovery system state.
     * @param kShutdown The shutdown system state.
     * @param kOffline The offline system state.
     */
    enum class SystemState : uint8_t {
        kUnknown        = 0, // Safety fallback
        kStartup        = 1,
        kIdle           = 2, // Completely stationary; safe to accept new paths
        kMoving         = 3, // Actively running an interpolator, velocity command, or jog stream
        kSettling       = 4, // After planning finished / Changing Control Mode / Stopping
        kError          = 5, // Safeguard stop triggered; hardware limits breached or E-stop
        kRecovery       = 6, // Resetting safety loops and clearing faults
        kShutdown       = 7, // Disabling amplifiers and powering down safely
        kOffline        = 8, // Hardware not found; will try to reconnect periodically
    };

    namespace DiagnosticFlags {    
        constexpr uint64_t kNone                         = 0; // No warning
        
        // --- Target Profile Saturation (Pre-Interpolation) ---
        constexpr uint64_t kTargetPosOutOfRange          = 1ULL << 1; // User target exceeds soft position limits, stop moving
        constexpr uint64_t kTargetVelOutOfRange          = 1ULL << 2; // User target exceeds soft velocity limits, stop moving
        constexpr uint64_t kTargetTorOutOfRange          = 1ULL << 3; // User target exceeds soft torque limits, stop moving
        constexpr uint64_t kTargetPosSaturation          = 1ULL << 4; // target clamped by soft position limits
        constexpr uint64_t kTargetVelSaturation          = 1ULL << 5; // target clamped by soft velocity limits
        constexpr uint64_t kTargetTorSaturation          = 1ULL << 6; // target clamped by soft torque limits
        
        // --- Real-Time Command Saturation (Post-PID Loop) ---
        constexpr uint64_t kActuatorPosSaturation        = 1ULL << 7; // Actuator command clamped by soft position limits
        constexpr uint64_t kActuatorVelSaturation        = 1ULL << 8; // Actuator command clamped by soft velocity limits
        constexpr uint64_t kActuatorTorSaturation        = 1ULL << 9; // Actuator command clamped by soft torque limits
        constexpr uint64_t kActuatorPosJumpSaturation    = 1ULL << 10; // Actuator command clamped by position jump limit
        constexpr uint64_t kActuatorVelJumpSaturation    = 1ULL << 11; // Actuator command clamped by velocity jump limit
        constexpr uint64_t kActuatorTorJumpSaturation    = 1ULL << 12; // Actuator command clamped by torque jump limit
        
        // --- Soft Workspace Boundary Safety Interceptions ---
        constexpr uint64_t kBoundaryVelClamp             = 1ULL << 13;  // Velocity zeroed in limit direction due to position boundary reached
        constexpr uint64_t kBoundaryJointImpedance       = 1ULL << 14; // Joint impedance applied due to position boundary reached
        
        // --- Algorithmic & Kinematic Planner Modifications ---
        constexpr uint64_t kPlanTimelineExtended         = 1ULL << 15; // Trajectory segment duration (dt) stretched for velocity limits
        constexpr uint64_t kPlanVelLimitInvalid          = 1ULL << 16; // Specified max velocity profile is smaller than the minimum allowed velocity
        constexpr uint64_t kPlanDeltaTooLarge            = 1ULL << 17; // Distance between points is too large for a dynamic timeline
        constexpr uint64_t kPlanVelocitySnap             = 1ULL << 18; // Large velocity shift over zero distance
        constexpr uint64_t kPlanPointSkipped             = 1ULL << 19; // Duplicated points skipped
        constexpr uint64_t kPlanInvalidSetting           = 1ULL << 20; // Invalid setting on Interpolator

        // --- Fault Flags ---
        constexpr uint64_t kFaultPosHardLimitReached     = 1ULL << 20;
        constexpr uint64_t kFaultVelHardLimitReached     = 1ULL << 21;
        constexpr uint64_t kFaultTorHardLimitReached     = 1ULL << 22;
        constexpr uint64_t kFaultPosTrackingFailed       = 1ULL << 23;
        constexpr uint64_t kFaultVelTrackingFailed       = 1ULL << 24;
        constexpr uint64_t kFaultTorTrackingFailed       = 1ULL << 25;
        
        constexpr uint64_t kFaultArmNotFound                = 1ULL << 26;
        constexpr uint64_t kFaultGripperNotFound            = 1ULL << 27;
        constexpr uint64_t kFaultButtonNotFound             = 1ULL << 28;
        constexpr uint64_t kFaultArmInitFailed              = 1ULL << 29;
        constexpr uint64_t kFaultGripperInitFailed          = 1ULL << 30;
        constexpr uint64_t kFaultButtonInitFailed           = 1ULL << 31;
        constexpr uint64_t kFaultHardwareEnableFailed       = 1ULL << 32;
        constexpr uint64_t kFaultHardwareChangeModeFailed   = 1ULL << 33;
        constexpr uint64_t kFaultHardwareSetReadFailed      = 1ULL << 34;

        constexpr uint64_t kFaultArmSizeMismatch            = 1ULL << 35;
        constexpr uint64_t kFaultGripperSizeMismatch        = 1ULL << 36;
        constexpr uint64_t kFaultArmJointSizeMismatch       = 1ULL << 37;
        constexpr uint64_t kFaultGripperJointSizeMismatch   = 1ULL << 38;
        constexpr uint64_t kFaultRobotNameMismatch          = 1ULL << 39;
        constexpr uint64_t kFaultArmControlStrategyMismatch = 1ULL << 40;

        constexpr uint64_t kFaultCollisionDetected          = 1ULL << 41;
        constexpr uint64_t kFaultRecoveryRequired           = 1ULL << 42;

        // --- Configuration Validation ---
        constexpr uint64_t kInvalidControlTypeCombination         = 1ULL << 43; // Control type combination is invalid or not supported
        constexpr uint64_t kCartesianControlRequirePositionTarget = 1ULL << 44; // Cartesian control require position target
        constexpr uint64_t kControlStrategyNotAvailable           = 1ULL << 45; // Control strategy is not available
        constexpr uint64_t kWaypointControlStrategyNotAllowed     = 1ULL << 46; // Waypoint control strategy is invalid
        constexpr uint64_t kWaypointTargetTypeNotAllowed          = 1ULL << 47; // Waypoint target type is invalid
        constexpr uint64_t kWaypointSmoothingMethodNotAllowed     = 1ULL << 48; // Waypoint smoothing method is invalid
        constexpr uint64_t kTeachingReplayControlRequirePositionTarget  = 1ULL << 49; // Teaching replay control require position target
        constexpr uint64_t kTeachingReplayControlStartPoseNotReachable  = 1ULL << 50; // Teaching replay control cannot reach start pose

        constexpr uint64_t kUnknown                               = 1ULL << 63;
    };

    // Explicit mapping of the operation's immediate result
    enum class CommandResponseStatus: uint8_t {
        kSuccess                    = 0, // Command accepted and executed
        kRejectedBlocked            = 1, // Control token is locked by another master client
        kRejectedConfigNotReady     = 2, // Configuration update in progress; current configuration is not yet active.
        kRejectedFaulted            = 3, // Robot is faulted and refuses standard motion commands
        kRejectedSequenceId         = 4, // Sequence ID is older than the last received sequence ID
        kRejectedConfigRequired     = 5, // Initial configuration missing; must set configuration once
        kInvalidCommand             = 6, // Packet payload failed structural or cryptographic validation
        kInvalidClientId            = 7, // Client ID not found in the static configuration registry
        kInvalidSessionId           = 8, // Session ID expired, unauthenticated, or mismatched
        kInvalidConfig              = 9, // Configuration is invalid
        kTimeout                    = 10, // Configuration update timed out
        kUnknown                    = 255
    };

    enum class MessageType: uint8_t{
        kSrvState = 0,
        kSdkCommandReq,
        kSdkCommandRes,
        kSdkConfigReq,
        kSdkConfigRes,
        kSdkHeartbeatReq,
        kSdkSafeguardReq,
        kSdkHandshakeReq,
        kSdkHandshakeRes,
        kSdkReleaseControlReq,
        kSdkReleaseControlRes,
        kSdkRecoveryReq,
        kSdkRecoveryRes,
    };

    enum class NetworkConfigAction : uint8_t {
        kGet = 0,
        kSetWifi = 1,
        kSetEthStatic = 2,
        kFactoryReset = 3,
    };
    

    // Bitmask for global machine system flags (Allows combining multiple states)
    namespace SystemFlag {
        constexpr uint16_t kHasControlToken     = 1 << 0; // 0x0001: This client currently owns the robot
        constexpr uint16_t kBlockedByOther      = 1 << 1; // 0x0002: Token is actively held by a competing client
        constexpr uint16_t kRecoveryRequired    = 1 << 2; // 0x0004: Safety system triggered (Needs Recovery/Reset)
        constexpr uint16_t kEmergencyStopActive = 1 << 3; // 0x0008: Physical hardware Emergency Stop is locked
        constexpr uint16_t kSimulationMode      = 1 << 4; // 0x0010: Robot running in digital simulation space
    }

    template<typename T>
    std::string enumToString(T x){
        static_assert(std::is_enum_v<T>, "enumToString: T must be an enum type");
        auto name = magic_enum::enum_name(x);
        return name.empty() ? "Unknown" : std::string(name.substr(1));
    }

    template<typename T>
    T stringToEnum(const std::string& s){
        static_assert(std::is_enum_v<T>, "stringToEnum: T must be an enum type");
        std::string s1 = "k" + s;
        return magic_enum::enum_cast<T>(s1).value_or(static_cast<T>(0));
    }

    template<typename T>
    std::vector<std::string> getAllEnumNames(){
        static_assert(std::is_enum_v<T>, "getAllEnumNames: T must be an enum type");

        constexpr auto names_view = magic_enum::enum_names<T>();
        std::vector<std::string> names;
        names.reserve(names_view.size());
        for (const auto& sv : names_view) {
            names.emplace_back(std::string(sv.substr(1)));
        }
        return names;
    }

    struct PositionTarget {
        float x, y, z;
    };

    struct CartesianTarget {
        float x, y, z;
        float qw, qx, qy, qz; //Quaternion
    };

    struct SinglePointTarget{
        float               interpolation_t; // Minimum interpolation time, t >= 0. May be extended based on interpolation_speed_ratio.
        float               interpolation_speed_ratio; // Ratio of soft joint limit, range [0, 1]

        float               gripper_joint[MAX_GRIPPER_SIZE][MAX_GRIPPER_JOINT_SIZE];

        /** Active member selected by payload.activated_control_strategy + enable_jog. */
        union {
            float               arm_joint[MAX_ARM_SIZE][MAX_ARM_JOINT_SIZE];

            /** @brief Arm tool pose relative to the base frame */
            CartesianTarget     arm_tool_cartesian[MAX_ARM_SIZE];

            JogCommand          arm_joint_jog[MAX_ARM_SIZE][MAX_ARM_JOINT_SIZE];

            /** 
            * @brief Relative operational space jog parameters.
            * @details Array index map: 0=X, 1=Y, 2=Z, 3=RX, 4=RY, 5=RZ. 
            *          Rotations [3-5] specify an Extrinsic XYZ Euler sequence evaluated 
            *          via sequential left-multiplication: R = R_x(RX) * R_y(RY) * R_z(RZ).
            *          Reference frames are set by `cartesian_jog_trans_frame` and `cartesian_jog_rot_frame`.
            */
            JogCommand          arm_cartesian_jog[MAX_ARM_SIZE][6];
        };
    };

    struct SdkCommandReq{
        // --- Header / Metadata ---
        uint32_t magic_header = MAGIC_HEADER;   //Prevent mis-interpretation of random data as valid command
        uint16_t client_id;
        uint32_t session_id;                    //Use to indicate the source of the command, as multiple clients may connect
        uint32_t sequence_id;                   //Sequence id of the command
        uint64_t timestamp_us;

        // --- Payload Block ---
        struct {
            char                robot_name[MAX_NAME_SIZE];
            uint8_t             arm_size = 0;
            uint8_t             gripper_size = 0;
            uint8_t             arm_joint_size[MAX_ARM_SIZE];
            uint8_t             gripper_joint_size[MAX_GRIPPER_SIZE];

            /** Echo of the configured strategy; not changed by SdkCommandReq (use SdkConfigReq). */
            ControlStrategy     activated_control_strategy[MAX_ARM_SIZE];
            TeachingCommand     teaching_cmd = TeachingCommand::kNone;
            uint8_t             enable_jog = 0;
            uint8_t             target_count = 0;
            SinglePointTarget   target[MAX_WAYPOINTS];
        }payload;

        uint8_t  security_hmac[HMAC_KEY_SIZE];     //Prevent unauthorized command injection, HMAC-MD5 of the payload using a pre-shared secret key
    };

    struct SdkCommandRes {
        // --- Header / Metadata ---
        uint32_t magic_header = MAGIC_HEADER;
        uint16_t request_client_id;
        uint32_t request_sequence_id;
        uint64_t request_received_us;
        uint64_t response_sent_us;

        // --- Payload Block ---
        struct {
            CommandResponseStatus status;
        }payload;

        uint8_t  security_hmac[HMAC_KEY_SIZE];
    };

    struct SdkConfigReq{
        // --- Header / Metadata ---
        uint32_t magic_header = MAGIC_HEADER;   //Prevent mis-interpretation of random data as valid command
        uint16_t client_id;
        uint32_t session_id;                    //Use to indicate the source of the command, as multiple clients may connect
        uint32_t sequence_id;                   //Sequence id of the command
        uint64_t timestamp_us;               

        // --- Payload Block ---
        struct {
            uint8_t                 read_only;

            ////////////////////If read_only is false, below fields are required//////////////////////
            char                    robot_name[MAX_NAME_SIZE];
            uint8_t                 simulation;
            
            uint8_t                 arm_size = 0;
            uint8_t                 gripper_size = 0;
            uint8_t                 arm_joint_size[MAX_ARM_SIZE];
            uint8_t                 gripper_joint_size[MAX_GRIPPER_SIZE];

            struct {
                ControlStrategy     control_strategy;
                SmoothingMethod     filter_type;
                ControlType         target_type;
                ControlType         actuator_mode;
                
                FrameReference      cartesian_jog_trans_frame = FrameReference::kTool;
                FrameReference      cartesian_jog_rot_frame = FrameReference::kTool;
                uint8_t             reset_control_mem;
                uint8_t             reset_interpolation;

                uint8_t             enable_joint[MAX_ARM_JOINT_SIZE];

                PositionTarget      tool_offset;
                float               soft_limit_position[MAX_ARM_JOINT_SIZE][2];
                float               soft_limit_velocity[MAX_ARM_JOINT_SIZE];
                float               soft_limit_torque[MAX_ARM_JOINT_SIZE];
            }arm[MAX_ARM_SIZE];

            struct {
                ControlType         target_type;
                float               soft_limit_position[MAX_GRIPPER_JOINT_SIZE][2];
            }gripper[MAX_GRIPPER_SIZE];
        }payload;

        uint8_t  security_hmac[HMAC_KEY_SIZE];     //Prevent unauthorized command injection, HMAC-MD5 of the payload using a pre-shared secret key
    };

    struct SdkConfigRes {
        // --- Header / Metadata ---
        uint32_t magic_header = MAGIC_HEADER;
        uint16_t request_client_id;
        uint32_t request_sequence_id;
        uint64_t request_received_us;
        uint64_t response_sent_us;

        // --- Payload Block ---
        struct {
            CommandResponseStatus   status;
            char                    robot_name[MAX_NAME_SIZE];
            uint8_t                 simulation;
            
            uint8_t                 arm_size = 0;
            uint8_t                 gripper_size = 0;
            uint8_t                 arm_joint_size[MAX_ARM_SIZE];
            uint8_t                 gripper_joint_size[MAX_GRIPPER_SIZE];

            struct {
                char                name[MAX_NAME_SIZE];
                ControlStrategy     control_strategy;
                SmoothingMethod     filter_type;
                ControlType         target_type;
                ControlType         actuator_mode;
                FrameReference      cartesian_jog_trans_frame = FrameReference::kTool; //Cartesian jog translation reference frame
                FrameReference      cartesian_jog_rot_frame = FrameReference::kTool; //Cartesian jog rotation reference frame

                PositionTarget      tool_offset;
                uint8_t             enabled_joint[MAX_ARM_JOINT_SIZE];
                float               soft_limit_position[MAX_ARM_JOINT_SIZE][2];
                float               soft_limit_velocity[MAX_ARM_JOINT_SIZE];
                float               soft_limit_torque[MAX_ARM_JOINT_SIZE];
                
                float               hard_limit_position[MAX_ARM_JOINT_SIZE][2];
                float               hard_limit_velocity[MAX_ARM_JOINT_SIZE];
                float               hard_limit_torque[MAX_ARM_JOINT_SIZE];

                float               follow_limit_position[MAX_ARM_JOINT_SIZE];
                float               follow_limit_velocity[MAX_ARM_JOINT_SIZE];
                float               follow_limit_torque[MAX_ARM_JOINT_SIZE];
                
                float               jump_limit_position[MAX_ARM_JOINT_SIZE];
                float               jump_limit_velocity[MAX_ARM_JOINT_SIZE];
                float               jump_limit_torque[MAX_ARM_JOINT_SIZE];
            }arm[MAX_ARM_SIZE];

            struct {
                char                name[MAX_NAME_SIZE];
                ControlType         target_type;
                float               soft_limit_position[MAX_GRIPPER_JOINT_SIZE][2];
                float               hard_limit_position[MAX_GRIPPER_JOINT_SIZE][2];
            }gripper[MAX_GRIPPER_SIZE];
        }payload;

        uint8_t  security_hmac[HMAC_KEY_SIZE];
    };

    struct SdkHeartbeatReq{
        // --- Header / Metadata ---
        uint32_t magic_header = MAGIC_HEADER;
        uint16_t client_id;
        uint32_t session_id;
        uint64_t timestamp_us;
        uint8_t  security_hmac[HMAC_KEY_SIZE];
    };

    struct SdkSafeguardReq{
        // --- Header / Metadata ---
        uint32_t magic_header = MAGIC_HEADER;
        uint16_t client_id;
        uint32_t session_id;
        uint64_t timestamp_us;
        uint8_t  security_hmac[HMAC_KEY_SIZE];
    };

    struct SdkHandshakeReq { //For client to request a session id from the server
        uint32_t magic_header = MAGIC_HEADER; 
        uint16_t client_id;                   
        uint32_t sequence_id;                 // Anti-replay counter
        uint16_t telemetry_port;              // Port for telemetry data
        uint64_t timestamp_us;                // Integrity timestamp
        uint8_t  security_hmac[HMAC_KEY_SIZE];
    };

    struct SdkHandshakeRes { //For server to respond with a session id to the client
        uint32_t magic_header = MAGIC_HEADER;
        uint16_t request_client_id;
        uint32_t request_sequence_id;
        uint64_t request_received_us;
        uint64_t response_sent_us;
        uint32_t assigned_session_id;
        
        // --- Payload Block ---
        struct {
            CommandResponseStatus status;
            /** Trailing handshake extension (optional on wire for old peers). */
            char robot_name[MAX_NAME_SIZE]{};
        }payload;
        
        uint8_t  security_hmac[HMAC_KEY_SIZE];
    };

    struct SdkReleaseControlReq {
        uint32_t magic_header = MAGIC_HEADER;
        uint16_t client_id;
        uint32_t session_id;          
        uint32_t sequence_id;                 // Anti-replay counter
        uint64_t timestamp_us;                // Integrity timestamp
        uint8_t  security_hmac[HMAC_KEY_SIZE];
    };
    
    struct SdkReleaseControlRes {
        uint32_t magic_header = MAGIC_HEADER;
        uint16_t request_client_id;
        uint32_t request_sequence_id;
        uint64_t request_received_us;
        uint64_t response_sent_us;

        // --- Payload Block ---
        struct {
            CommandResponseStatus status;
        }payload;
        
        uint8_t  security_hmac[HMAC_KEY_SIZE];
    };

    struct SdkRecoveryReq {
        uint32_t magic_header = MAGIC_HEADER;
        uint16_t client_id;
        uint32_t session_id;                   
        uint32_t sequence_id;                 // Anti-replay counter
        uint64_t timestamp_us;                // Integrity timestamp
        uint8_t  security_hmac[HMAC_KEY_SIZE];
    };
    
    struct SdkRecoveryRes {
        uint32_t magic_header = MAGIC_HEADER;
        uint16_t request_client_id;
        uint32_t request_sequence_id;
        uint64_t request_received_us;
        uint64_t response_sent_us;

        // --- Payload Block ---
        struct {
            CommandResponseStatus status;
        }payload;
        
        uint8_t  security_hmac[HMAC_KEY_SIZE];
    };
    
    using CoreRequestVariant = std::variant<std::monostate, SdkCommandReq, SdkConfigReq, SdkHandshakeReq, SdkReleaseControlReq, SdkRecoveryReq>;
    using CoreResponseVariant = std::variant<std::monostate, SdkCommandRes, SdkConfigRes, SdkHandshakeRes, SdkReleaseControlRes, SdkRecoveryRes>;
    using MonitoringRequestVariant = std::variant<std::monostate, SdkHeartbeatReq, SdkSafeguardReq>;
    using CoreRequestVariantPtr = std::unique_ptr<CoreRequestVariant>;
    using CoreResponseVariantPtr = std::shared_ptr<CoreResponseVariant>;
    using MonitoringRequestVariantPtr = std::unique_ptr<MonitoringRequestVariant>;

    struct SrvState{
        // --- Header / Metadata ---
        uint32_t magic_header = MAGIC_HEADER;   //Prevent mis-interpretation of random data as valid command
        uint16_t client_id = 0;                 //0 for multicast
        uint32_t sequence_id;                   //Sequence id of the state
        uint16_t current_client_id;             //Client id of the current client
        uint32_t current_client_sequence_id;    //Sequence id of the current client
        uint64_t timestamp_us;               

        // --- Payload Block ---
        struct {
            char                    robot_name[MAX_NAME_SIZE];
            SystemState             system_state;       // Idle, Running, Paused
            PlanResult              plan_result;
            uint64_t                system_diagnostic_flags; 
            uint32_t                session_id;  // Proves WHICH client is currently controlling it
            uint32_t                last_processed_seq; // Matches the client's SdkCommandReq sequence_id

            uint8_t                 arm_size = 0;
            uint8_t                 gripper_size = 0;
            uint8_t                 arm_joint_size[MAX_ARM_SIZE];
            uint8_t                 gripper_joint_size[MAX_GRIPPER_SIZE];

            struct {
                float               position[MAX_ARM_JOINT_SIZE];
                float               velocity[MAX_ARM_JOINT_SIZE];
                float               torque[MAX_ARM_JOINT_SIZE];
                float               acceleration[MAX_ARM_JOINT_SIZE];
                CartesianTarget     joint_poses[MAX_ARM_JOINT_SIZE]; //Included ee pose as the last one
                CartesianTarget     tool_pose;
                float               last_set_command[MAX_ARM_JOINT_SIZE];
                uint64_t            diagnostic_flags[MAX_ARM_JOINT_SIZE];
            }arm[MAX_ARM_SIZE];

            struct {
                float               position[MAX_GRIPPER_JOINT_SIZE];
                float               last_set_command[MAX_GRIPPER_JOINT_SIZE];
                uint64_t            diagnostic_flags[MAX_GRIPPER_JOINT_SIZE];
            }gripper[MAX_GRIPPER_SIZE];
        }payload;
        
        uint8_t  security_hmac[HMAC_KEY_SIZE];     //Prevent unauthorized command injection, HMAC-MD5 of the payload using a pre-shared secret key
    };

////////////////////////////////////////Use in rt_control & robotics////////////////////////////////////////
    struct RobotState{
        //Arm related
        int     arm_size;
        int     arm_joint_size[MAX_ARM_SIZE];
        float   arm_joint_pos[MAX_ARM_SIZE][MAX_ARM_JOINT_SIZE];
        float   arm_joint_vel[MAX_ARM_SIZE][MAX_ARM_JOINT_SIZE];
        float   arm_joint_cur[MAX_ARM_SIZE][MAX_ARM_JOINT_SIZE];
        float   arm_joint_tor[MAX_ARM_SIZE][MAX_ARM_JOINT_SIZE];

        //Gripper related
        int     gripper_size;
        int     gripper_joint_size[MAX_GRIPPER_SIZE];
        float   gripper_joint_pos[MAX_GRIPPER_SIZE][MAX_GRIPPER_JOINT_SIZE];

        //Joint force sensor related
        bool    enable_joint_torque_sensor;
        float   joint_torque_sensor[MAX_ARM_SIZE][MAX_ARM_JOINT_SIZE];

        //Arm encoder related
        float   encoder_initial_position[MAX_ARM_SIZE][MAX_ARM_JOINT_SIZE];
        float   encoder_saved_position[MAX_ARM_SIZE][MAX_ARM_JOINT_SIZE];
        float   encoder_offset[MAX_ARM_SIZE][MAX_ARM_JOINT_SIZE];
    };

    struct PlannerCommand{
        bool read_only = true;

        ControlType arm_command_mode;
        uint8_t     arm_size;
        uint8_t     arm_joint_size[MAX_ARM_SIZE];
        float       arm_joint_target[MAX_ARM_SIZE][MAX_ARM_JOINT_SIZE];

        uint8_t     gripper_size;
        uint8_t     gripper_joint_size[MAX_GRIPPER_SIZE];
        float       gripper_joint_target[MAX_GRIPPER_SIZE][MAX_GRIPPER_JOINT_SIZE];
    };

    struct PlannerState{
        float target_before_interpolation[MAX_ARM_SIZE][MAX_ARM_JOINT_SIZE];
        float sliced_tar[MAX_ARM_SIZE][MAX_ARM_JOINT_SIZE];
        float task_target_before_interpolation[MAX_ARM_SIZE][6];
        float task_sliced_tar[MAX_ARM_SIZE][6];
        float task_pose[MAX_ARM_SIZE][6];
        float pid_cmd[MAX_ARM_SIZE][MAX_ARM_JOINT_SIZE];
        float guard_pos_cmd[MAX_ARM_SIZE][MAX_ARM_JOINT_SIZE];
        float sat_cmd[MAX_ARM_SIZE][MAX_ARM_JOINT_SIZE];
        float jump_cmd[MAX_ARM_SIZE][MAX_ARM_JOINT_SIZE];
        
        float joint_gravity[MAX_ARM_SIZE][MAX_ARM_JOINT_SIZE];
        float gravity_kd_effect[MAX_ARM_SIZE][MAX_ARM_JOINT_SIZE];
        float filtered_joint_vel[MAX_ARM_SIZE][MAX_ARM_JOINT_SIZE];
        float friction[MAX_ARM_SIZE][MAX_ARM_JOINT_SIZE];
        float joint_impedance[MAX_ARM_SIZE][MAX_ARM_JOINT_SIZE];

        float body_forces[6];
    };

//////////////////////////////////// Use in cuarm_upper_software - remote page////////////////////////////////////
    enum class PanelRemoteState: uint8_t{kStop, kStart};  
    struct PanelState{
        PanelRemoteState state;

        //Arm related
        uint8_t arm_size;
        uint8_t arm_joint_size[MAX_ARM_SIZE];
        float arm_joint_pos[MAX_ARM_SIZE][MAX_ARM_JOINT_SIZE];
        float arm_joint_vel[MAX_ARM_SIZE][MAX_ARM_JOINT_SIZE];
        float arm_joint_tor[MAX_ARM_SIZE][MAX_ARM_JOINT_SIZE];
        float arm_ee_pose[MAX_ARM_SIZE][7];
        float arm_tool_pose[MAX_ARM_SIZE][7];
        float arm_interpolated_target[MAX_ARM_SIZE][MAX_ARM_JOINT_SIZE];

        //Gripper related
        uint8_t gripper_size;
        uint8_t gripper_joint_size[MAX_GRIPPER_SIZE];
        float   gripper_joint_pos[MAX_GRIPPER_SIZE][MAX_GRIPPER_JOINT_SIZE];
    };

    struct RemotePanelCommand{
        ControlType target_type;
        ControlType actuator_mode;
        SmoothingMethod interpolation_type;
    
        //Arm related
        uint8_t arm_size = 0;
        uint8_t arm_joint_size[MAX_ARM_SIZE];
        uint8_t arm_target_count = 0;
        float   arm_interpolation_t[MAX_WAYPOINTS];
        float   arm_interpolation_speed_ratio[MAX_WAYPOINTS];
        float   arm_joint_cmd[MAX_WAYPOINTS][MAX_ARM_SIZE][MAX_ARM_JOINT_SIZE];
    
        //Gripper related  
        uint8_t gripper_size = 0;
        uint8_t gripper_joint_size[MAX_GRIPPER_SIZE];
        float   gripper_joint_cmd[MAX_GRIPPER_SIZE][MAX_GRIPPER_JOINT_SIZE];
    };

//////////////////////////////////// Helper functions ///////////////////////////////////////
    void print(const SdkCommandReq& value);
    void print(const SdkCommandRes& value);
    void print(const SdkConfigReq& value);
    void print(const SdkConfigRes& value);
    void print(const SdkHandshakeReq& value);
    void print(const SdkHandshakeRes& value);
    void print(const SdkHeartbeatReq& value);
    void print(const SdkSafeguardReq& value);
    void print(const SrvState& value);
    std::vector<std::string> diagnosticFlagsToString(uint64_t flags);
    void printDiagnosticFlags(uint64_t flags);
    std::pair<std::string, std::string> getStatusErrorMessage(CommandResponseStatus status);

    // //Get initial request
    SdkConfigReq getReadOnlyConfigRequest(uint16_t client_id, uint32_t session_id, uint32_t sequence_id);
    SdkCommandReq getInitialCommandRequest(uint16_t client_id, uint32_t session_id, uint32_t sequence_id);
    void updateConfigRequest(const SdkConfigRes& response, SdkConfigReq& request);

    // C++17 Type Trait to check for a .resize() member function
    template<typename T, typename = void>
    struct has_resize : std::false_type {};

    template<typename T>
    struct has_resize<T, std::void_t<decltype(std::declval<T&>().resize(std::declval<size_t>()))>> : std::true_type {};

    template<typename T>
    void writeCartesian(const T& pose7, CartesianTarget& out) {
        // Generic element accessor that handles arrays, pointers, and Eigen reference objects
        auto get_element = [&pose7](size_t index) {
            if constexpr (std::is_pointer_v<std::decay_t<T>> || std::is_array_v<std::remove_reference_t<T>>) {
                return pose7[index];
            } else {
                return pose7(index); // Safely indexes Eigen vectors via ()
            }
        };

        // Extract underlying Scalar type to ensure safety
        using Scalar = std::decay_t<decltype(get_element(0))>;
        static_assert(std::is_same_v<Scalar, float> || std::is_same_v<Scalar, double>,
                    "writeCartesian: pose7 element type must be float or double");

        out.x  = static_cast<float>(get_element(0));
        out.y  = static_cast<float>(get_element(1));
        out.z  = static_cast<float>(get_element(2));
        out.qw = static_cast<float>(get_element(3));
        out.qx = static_cast<float>(get_element(4));
        out.qy = static_cast<float>(get_element(5));
        out.qz = static_cast<float>(get_element(6));
    }

    template<typename T>
    inline void readCartesianTarget(const CartesianTarget& cartesian, T& out) {
        // Safe C++17 alternative to C++20 concepts/requires
        if constexpr (has_resize<T>::value) {
            out.resize(7);
        }

        // Generic setter mapping cleanly to raw buffers or Eigen objects
        auto set_element = [&out](size_t index, auto value) {
            if constexpr (std::is_pointer_v<std::decay_t<T>> || std::is_array_v<std::remove_reference_t<T>>) {
                using ElementType = std::remove_reference_t<decltype(out[0])>;
                out[index] = static_cast<ElementType>(value);
            } else {
                using ScalarType = typename std::decay_t<T>::Scalar;
                out(index) = static_cast<ScalarType>(value);
            }
        };

        set_element(0, cartesian.x);
        set_element(1, cartesian.y);
        set_element(2, cartesian.z);
        set_element(3, cartesian.qw);
        set_element(4, cartesian.qx);
        set_element(5, cartesian.qy);
        set_element(6, cartesian.qz);
    }
};

#endif