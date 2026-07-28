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
    #define MAGIC_HEADER            0x484B434C // "HKCL" in ASCII
    #ifndef HMAC_KEY_SIZE
        #define HMAC_KEY_SIZE           16U
    #endif

    enum class ControlStrategy: uint8_t{
        kJoint = 0, 
        kCartesian, 
        kCartesianLine, 
        kNullspace, 
        kGravityCompensation,
        kRecord,
        kPlayback,
    };

    enum class FrameReference: uint8_t { kTool, kBase };

    enum class SmoothingMethod: uint8_t{
        kLinear,
        kCos,
        kCubic,
        kQuintic,
        kQuinticPath,
        kLowPassFilter,
        kNone,
    };

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

    enum class ControlType: uint8_t{
        kPosition=0, 
        kVelocity, 
        kTorque,
        kInvalid=255,
    };

    enum class CartesianLineMoveStrategy : uint8_t  {
        kRejectEntirely = 0, // Abort the move immediately; do not start moving.
        kStopAtBoundary = 1, // Move and stop smoothly at the last reachable point before the failure.
        kDeviateAndBypassing = 2, // Deviate from the line (e.g., switch to joint space) to bypass the unreachable zone.
        kSegmentedExecution = 3 // Execute valid parts, skip the bad segment, and resume line tracking later.
    };
    enum class PlaybackState: uint8_t{kStop, kStart, kReset};

    /** Per-axis jog request when SdkCommandReq.payload.enable_jog is set. */
    enum class JogCommand: uint8_t {
        kStop = 0,
        kIncrease = 1,
        kDecrease = 2,
    };
    
    enum class PlanResult : uint8_t {
        kSuccess                    = 0,  // IK passed, time allocation valid, safe to execute
        kPoseNotReachable           = 1,  // Target 6D pose is physically outside the workspace
        kLinearPathFailed           = 2,  // Reachable target, but continuous linear path is blocked (Singularity / Joint Limit)
    };

    enum class SystemState : uint8_t {
        kUnknown        = 0, // Safety fallback
        kStartup        = 1,
        kIdle           = 2, // Completely stationary; safe to accept new paths
        kMoving         = 3, // Actively running an interpolator, velocity command, or jog stream
        kSettling       = 4, // After planning finished / Changing Control Mode / Stopping
        kError          = 5, // Safeguard stop triggered; hardware limits breached or E-stop
        kRecovery       = 6, // Resetting safety loops and clearing faults
        kShutdown       = 7, // Disabling amplifiers and powering down safely
    };

    namespace DiagnosticFlags {    
        constexpr uint64_t kNone                         = 0; // No warning
        
        // --- Target Profile Saturation (Pre-Interpolation) ---
        constexpr uint64_t kTargetPosSaturation          = 1 << 0; // User target clamped by soft position limits
        constexpr uint64_t kTargetVelSaturation          = 1 << 1; // User target clamped by soft velocity limits
        constexpr uint64_t kTargetTorSaturation          = 1 << 2; // User target clamped by soft torque limits
        
        // --- Real-Time Command Saturation (Post-PID Loop) ---
        constexpr uint64_t kActuatorPosSaturation        = 1 << 3; // Actuator command clamped by soft position limits
        constexpr uint64_t kActuatorVelSaturation        = 1 << 4; // Actuator command clamped by soft velocity limits
        constexpr uint64_t kActuatorTorSaturation        = 1 << 5; // Actuator command clamped by soft torque limits
        constexpr uint64_t kActuatorPosJumpSaturation    = 1 << 6; // Actuator command clamped by position jump limit
        constexpr uint64_t kActuatorVelJumpSaturation    = 1 << 7; // Actuator command clamped by velocity jump limit
        constexpr uint64_t kActuatorTorJumpSaturation    = 1 << 8; // Actuator command clamped by torque jump limit
        
        // --- Soft Workspace Boundary Safety Interceptions ---
        constexpr uint64_t kBoundaryVelClamp             = 1 << 9;  // Velocity zeroed in limit direction due to position boundary reached
        constexpr uint64_t kBoundaryJointImpedance       = 1 << 10; // Joint impedance applied due to position boundary reached
        
        // --- Algorithmic & Kinematic Planner Modifications ---
        constexpr uint64_t kPlanTimelineExtended         = 1 << 11; // Trajectory segment duration (dt) stretched for velocity limits
        constexpr uint64_t kPlanVelLimitInvalid          = 1 << 12; // Specified max velocity profile is smaller than the minimum allowed velocity
        constexpr uint64_t kPlanDeltaTooLarge            = 1 << 13; // Distance between points is too large for a dynamic timeline
        constexpr uint64_t kPlanVelocitySnap             = 1 << 14; // Large velocity shift over zero distance
        constexpr uint64_t kPlanPointSkipped             = 1 << 15; // Duplicated points skipped

        // --- Fault Flags ---
        constexpr uint64_t kFaultPosHardLimitReached     = 1 << 16;
        constexpr uint64_t kFaultVelHardLimitReached     = 1 << 17;
        constexpr uint64_t kFaultTorHardLimitReached     = 1 << 18;
        constexpr uint64_t kFaultPosTrackingFailed       = 1 << 19;
        constexpr uint64_t kFaultVelTrackingFailed       = 1 << 20;
        constexpr uint64_t kFaultTorTrackingFailed       = 1 << 21;
        
        constexpr uint64_t kFaultArmNotFound             = 1 << 22;
        constexpr uint64_t kFaultGripperNotFound         = 1 << 23;
        constexpr uint64_t kFaultHardwareInitFailed      = 1 << 24;

        constexpr uint64_t kFaultArmSizeMismatch            = 1 << 25;
        constexpr uint64_t kFaultGripperSizeMismatch        = 1 << 26;
        constexpr uint64_t kFaultArmJointSizeMismatch       = 1 << 27;
        constexpr uint64_t kFaultGripperJointSizeMismatch   = 1 << 28;
        constexpr uint64_t kFaultRobotNameMismatch          = 1 << 29;

        constexpr uint64_t kFaultCollisionDetected          = 1 << 30;
        constexpr uint64_t kFaultRecoveryRequired           = 1 << 31; // Recovery required

        // --- Configuration Validation ---
        constexpr uint64_t kInvalidControlTypeCombination         = 1ULL << 32; // Control type combination is invalid or not supported
        constexpr uint64_t kCartesianControlRequirePositionTarget = 1ULL << 33; // Cartesian control require position target
        constexpr uint64_t kControlStrategyNotAvailable           = 1ULL << 34; // Control strategy is not available
        constexpr uint64_t kWaypointControlStrategyNotAllowed     = 1ULL << 35; // Waypoint control strategy is invalid
        constexpr uint64_t kWaypointTargetTypeNotAllowed          = 1ULL << 36; // Waypoint target type is invalid
        constexpr uint64_t kWaypointSmoothingMethodNotAllowed     = 1ULL << 37; // Waypoint smoothing method is invalid
        constexpr uint64_t kPlaybackControlRequirePositionTarget  = 1ULL << 38; // Playback control require position target
        constexpr uint64_t kPlaybackControlStartPoseNotReachable  = 1ULL << 39; // Playback control cannot reach start pose

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
        float qw, qx, qy, qz;
    };

    struct SinglePointTarget{
        float               interpolation_t;
        float               interpolation_speed_ratio;

        float               gripper_joint[MAX_ARM_SIZE][MAX_GRIPPER_JOINT_SIZE];

        /** Active member selected by payload.activated_control_strategy + enable_jog. */
        union {
            float               arm_joint[MAX_ARM_SIZE][MAX_ARM_JOINT_SIZE];
            CartesianTarget     arm_tool_cartesian[MAX_ARM_SIZE];
            JogCommand          arm_joint_jog[MAX_ARM_SIZE][MAX_ARM_JOINT_SIZE];
            JogCommand          arm_cartesian_jog[MAX_ARM_SIZE][6]; // x,y,z, RX,RY,RZ (frame from SdkConfigReq.arm[].frame_reference)
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
            ControlStrategy     activated_control_strategy = ControlStrategy::kJoint;
            uint8_t             enable_jog = 0;  // 1: jog arm of union; 0: absolute arm
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
                PlaybackState       playback_cmd;
                
                FrameReference      frame_reference = FrameReference::kTool;
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
                PlaybackState       playback_cmd;
                FrameReference      frame_reference = FrameReference::kTool;

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

//////////////////////////////////// Use in cuarm_upper_software////////////////////////////////////
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
    void print_diagnostic_flags(uint64_t flags);
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