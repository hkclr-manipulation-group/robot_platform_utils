#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <yaml-cpp/yaml.h>

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace robot::platform {

class YamlConfigParseError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

struct JointSoftLimitConfig {
    std::vector<float> position_min_deg;
    std::vector<float> position_max_deg;
    std::vector<float> velocity;
    std::vector<float> acceleration;
    std::vector<float> torque;
    bool has_position = false;
    bool has_velocity = false;
    bool has_acceleration = false;
    bool has_torque = false;
};

struct JointJumpLimitConfig {
    std::vector<float> position;
    std::vector<float> velocity;
    std::vector<float> torque;
};

struct TaskMotionLimitConfig {
    std::array<float, 3> linear_velocity{};
    std::array<float, 3> linear_acceleration{};
    float angular_velocity = 0.f;
    float angular_acceleration = 0.f;
};

struct ArmPidConfig {
    std::vector<double> pos_vel_kp;
    std::vector<double> pos_tor_kp;
    std::vector<double> vel_tor_kp;
    std::vector<double> pos_vel_kd;
    std::vector<double> pos_tor_kd;
    std::vector<double> vel_tor_kd;
};

struct ArmGravityControlConfig {
    std::vector<float> ratio;
    std::vector<float> kd;
};

struct ArmJointImpedanceConfig {
    std::vector<float> kp;
    std::vector<float> kd;
    std::vector<float> torque_saturation;
};

struct ArmFrictionCoulombViscousConfig {
    std::vector<float> ratio;
    std::vector<float> coulumb;
    std::vector<float> viscous;
};

struct ArmFrictionConfig {
    ArmFrictionCoulombViscousConfig coulumb_with_viscous;
};

struct ArmForceControlConfig {
    ArmGravityControlConfig gravity;
    ArmJointImpedanceConfig joint_impedance;
    std::vector<float> velocity_low_pass_filter_ratio;
    ArmFrictionConfig friction;
};

struct ArmSafetyConfig {
    std::optional<JointSoftLimitConfig> joint_soft_limit;
    std::optional<JointJumpLimitConfig> joint_jump;
    std::optional<TaskMotionLimitConfig> task_motion_limit;
};

struct ArmConfig {
    std::string name;
    std::string type;
    int joint_size = 0;
    std::vector<std::string> urdf_joints;
    std::vector<std::string> urdf_links;
    std::vector<bool> enable_joint;
    std::optional<ArmPidConfig> pid;
    std::optional<ArmSafetyConfig> safety;
    std::optional<ArmForceControlConfig> force_control;
};

struct GripperConfig {
    int joint_size = 0;
    std::array<float, 3> offset_from_ee{};
};

struct RobotSectionConfig {
    std::string name;
    std::string urdf;
    std::vector<ArmConfig> arms;
    std::vector<GripperConfig> grippers;
};

struct SelfCollisionDetectionConfig {
    bool enable = false;
    bool use_curobo_spheres = false;
    float buffer_boundary = 0.f;
};

struct RtControlConfig {
    float dt_control_ms = 0.f;
    std::string robotics_library;
    std::string actuator_mode_under_position_control;
    std::optional<SelfCollisionDetectionConfig> self_collision_detection;
};

/** Strongly typed view of robot `config.yaml`. */
struct RobotConfigDocument {
    std::string model;
    std::optional<RobotSectionConfig> robot;
    std::optional<RtControlConfig> rt_control;
    YAML::Node curobo_yaml;
    bool has_curobo_yaml = false;

    bool empty() const {
        return model.empty() && !robot.has_value() && !rt_control.has_value();
    }
};

template <typename T>
const T& requireField(const std::optional<T>& value, const char* context) {
    if (!value.has_value()) {
        throw YamlConfigParseError(std::string("Missing required config field: ") + context);
    }
    return *value;
}

const RobotSectionConfig& requireRobotSection(const RobotConfigDocument& document);
const ArmConfig& requireArm(const RobotConfigDocument& document, int arm_index);
float controlDtSeconds(const RobotConfigDocument& document);
const std::string& roboticsLibraryName(const RobotConfigDocument& document);

void clearRobotConfigCache();

RobotConfigDocument parseRobotConfigDocument(const YAML::Node& root);
RobotConfigDocument loadRobotConfigDocument(const std::string& yaml_path);
std::optional<RobotConfigDocument> tryLoadRobotConfigFromPrefix(
    const std::string& config_prefix_path);

std::string mapYamlArmTypeToInternal(const std::string& yaml_type);

}  // namespace robot::platform

#endif
