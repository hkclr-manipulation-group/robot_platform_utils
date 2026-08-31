#include "robot_config.h"

#include "config_loader.h"

#include <filesystem>
#include <mutex>
#include <sstream>
#include <unordered_map>

namespace robot::platform {
namespace {

struct RobotConfigCacheEntry {
    std::filesystem::file_time_type mtime{};
    RobotConfigDocument document;
};

std::mutex g_robot_config_cache_mutex;
std::unordered_map<std::string, RobotConfigCacheEntry> g_robot_config_cache;

[[noreturn]] void throwParseError(const std::string& context, const std::string& message) {
    throw YamlConfigParseError(context + ": " + message);
}

void requireNode(
    const YAML::Node& node,
    const std::string& context,
    YAML::NodeType::value type) {
    if (!node || node.Type() != type) {
        throwParseError(context, "missing or invalid node");
    }
}

template <typename T>
T readScalar(
    const YAML::Node& node,
    const std::string& context) {
    if (!node || !node.IsScalar()) {
        throwParseError(context, "expected scalar");
    }
    try {
        return node.as<T>();
    } catch (const YAML::Exception& e) {
        throwParseError(context, e.what());
    }
}

std::vector<std::string> readStringSequence(
    const YAML::Node& node,
    const std::string& context) {
    requireNode(node, context, YAML::NodeType::Sequence);
    std::vector<std::string> values;
    values.reserve(node.size());
    for (std::size_t i = 0; i < node.size(); ++i) {
        values.push_back(readScalar<std::string>(node[i], context + "[" + std::to_string(i) + "]"));
    }
    return values;
}

std::vector<bool> readBoolSequence(
    const YAML::Node& node,
    const std::string& context) {
    requireNode(node, context, YAML::NodeType::Sequence);
    std::vector<bool> values;
    values.reserve(node.size());
    for (std::size_t i = 0; i < node.size(); ++i) {
        values.push_back(readScalar<bool>(node[i], context + "[" + std::to_string(i) + "]"));
    }
    return values;
}

std::vector<double> readDoubleSequence(
    const YAML::Node& node,
    const std::string& context) {
    requireNode(node, context, YAML::NodeType::Sequence);
    std::vector<double> values;
    values.reserve(node.size());
    for (std::size_t i = 0; i < node.size(); ++i) {
        values.push_back(readScalar<double>(node[i], context + "[" + std::to_string(i) + "]"));
    }
    return values;
}

std::vector<float> readFloatVector(
    const YAML::Node& node,
    const std::string& context) {
    requireNode(node, context, YAML::NodeType::Sequence);
    std::vector<float> values;
    values.reserve(node.size());
    for (std::size_t i = 0; i < node.size(); ++i) {
        values.push_back(readScalar<float>(node[i], context + "[" + std::to_string(i) + "]"));
    }
    return values;
}

void readFloatArray(
    const YAML::Node& node,
    const std::string& context,
    std::array<float, 6>& out) {
    requireNode(node, context, YAML::NodeType::Sequence);
    const std::size_t count = std::min<std::size_t>(6, node.size());
    for (std::size_t i = 0; i < count; ++i) {
        out[i] = readScalar<float>(node[i], context + "[" + std::to_string(i) + "]");
    }
}

void readFloatArray3(
    const YAML::Node& node,
    const std::string& context,
    std::array<float, 3>& out) {
    requireNode(node, context, YAML::NodeType::Sequence);
    const std::size_t count = std::min<std::size_t>(3, node.size());
    for (std::size_t i = 0; i < count; ++i) {
        out[i] = readScalar<float>(node[i], context + "[" + std::to_string(i) + "]");
    }
}

JointSoftLimitConfig parseJointSoftLimitConfig(
    const YAML::Node& node,
    const std::string& context) {
    requireNode(node, context, YAML::NodeType::Map);
    JointSoftLimitConfig limits;

    const YAML::Node position = node["position"];
    if (position) {
        requireNode(position, context + ".position", YAML::NodeType::Sequence);
        if (position.size() < 2) {
            throwParseError(context + ".position", "expected [min, max] sequences");
        }
        limits.position_min_deg =
            readFloatVector(position[0], context + ".position[0]");
        limits.position_max_deg =
            readFloatVector(position[1], context + ".position[1]");
        limits.has_position = true;
    }
    if (node["velocity"]) {
        limits.velocity = readFloatVector(node["velocity"], context + ".velocity");
        limits.has_velocity = true;
    }
    if (node["acceleration"]) {
        limits.acceleration =
            readFloatVector(node["acceleration"], context + ".acceleration");
        limits.has_acceleration = true;
    }
    if (node["torque"]) {
        limits.torque = readFloatVector(node["torque"], context + ".torque");
        limits.has_torque = true;
    }
    return limits;
}

JointJumpLimitConfig parseJointJumpLimitConfig(
    const YAML::Node& node,
    const std::string& context) {
    requireNode(node, context, YAML::NodeType::Map);
    JointJumpLimitConfig limits;
    if (node["position"]) {
        limits.position = readFloatVector(node["position"], context + ".position");
    }
    if (node["velocity"]) {
        limits.velocity = readFloatVector(node["velocity"], context + ".velocity");
    }
    if (node["torque"]) {
        limits.torque = readFloatVector(node["torque"], context + ".torque");
    }
    return limits;
}

TaskMotionLimitConfig parseTaskMotionLimitConfig(
    const YAML::Node& node,
    const std::string& context) {
    requireNode(node, context, YAML::NodeType::Map);
    TaskMotionLimitConfig limits;
    if (node["linear_velocity"]) {
        readFloatArray3(
            node["linear_velocity"], context + ".linear_velocity", limits.linear_velocity);
    }
    if (node["linear_acceleration"]) {
        readFloatArray3(
            node["linear_acceleration"],
            context + ".linear_acceleration",
            limits.linear_acceleration);
    }
    if (node["angular_velocity"]) {
        limits.angular_velocity =
            readScalar<float>(node["angular_velocity"], context + ".angular_velocity");
    }
    if (node["angular_acceleration"]) {
        limits.angular_acceleration =
            readScalar<float>(node["angular_acceleration"], context + ".angular_acceleration");
    }
    return limits;
}

ArmPidConfig parseArmPidConfig(
    const YAML::Node& node,
    const std::string& context) {
    requireNode(node, context, YAML::NodeType::Map);
    ArmPidConfig pid;
    pid.pos_vel_kp = readDoubleSequence(node["pos_vel_kp"], context + ".pos_vel_kp");
    pid.pos_tor_kp = readDoubleSequence(node["pos_tor_kp"], context + ".pos_tor_kp");
    pid.vel_tor_kp = readDoubleSequence(node["vel_tor_kp"], context + ".vel_tor_kp");
    pid.pos_vel_kd = readDoubleSequence(node["pos_vel_kd"], context + ".pos_vel_kd");
    pid.pos_tor_kd = readDoubleSequence(node["pos_tor_kd"], context + ".pos_tor_kd");
    pid.vel_tor_kd = readDoubleSequence(node["vel_tor_kd"], context + ".vel_tor_kd");
    return pid;
}

ArmForceControlConfig parseArmForceControlConfig(
    const YAML::Node& node,
    const std::string& context) {
    requireNode(node, context, YAML::NodeType::Map);
    ArmForceControlConfig force_control;
    if (node["gravity"]) {
        const YAML::Node gravity = node["gravity"];
        force_control.gravity.ratio =
            readFloatVector(gravity["ratio"], context + ".gravity.ratio");
        force_control.gravity.kd =
            readFloatVector(gravity["kd"], context + ".gravity.kd");
    }
    if (node["joint_impedance"]) {
        const YAML::Node impedance = node["joint_impedance"];
        force_control.joint_impedance.kp =
            readFloatVector(impedance["kp"], context + ".joint_impedance.kp");
        force_control.joint_impedance.kd =
            readFloatVector(impedance["kd"], context + ".joint_impedance.kd");
        force_control.joint_impedance.torque_saturation = readFloatVector(
            impedance["torque_saturation"],
            context + ".joint_impedance.torque_saturation");
    }
    if (node["velocity_low_pass_filter_ratio"]) {
        force_control.velocity_low_pass_filter_ratio = readFloatVector(
            node["velocity_low_pass_filter_ratio"],
            context + ".velocity_low_pass_filter_ratio");
    }
    if (node["friction"]) {
        const YAML::Node friction = node["friction"]["coulumb_with_viscous"];
        force_control.friction.coulumb_with_viscous.ratio =
            readFloatVector(friction["ratio"], context + ".friction.coulumb_with_viscous.ratio");
        force_control.friction.coulumb_with_viscous.coulumb =
            readFloatVector(friction["coulumb"], context + ".friction.coulumb_with_viscous.coulumb");
        force_control.friction.coulumb_with_viscous.viscous =
            readFloatVector(friction["viscous"], context + ".friction.coulumb_with_viscous.viscous");
    }
    return force_control;
}

ArmSafetyConfig parseArmSafetyConfig(
    const YAML::Node& node,
    const std::string& context) {
    requireNode(node, context, YAML::NodeType::Map);
    ArmSafetyConfig safety;
    if (node["joint_soft_limit"]) {
        safety.joint_soft_limit = parseJointSoftLimitConfig(
            node["joint_soft_limit"], context + ".joint_soft_limit");
    }
    if (node["joint_jump"]) {
        safety.joint_jump = parseJointJumpLimitConfig(
            node["joint_jump"], context + ".joint_jump");
    }
    if (node["task_motion_limit"]) {
        safety.task_motion_limit = parseTaskMotionLimitConfig(
            node["task_motion_limit"], context + ".task_motion_limit");
    }
    return safety;
}

ArmConfig parseArmConfig(
    const YAML::Node& node,
    const std::string& context) {
    requireNode(node, context, YAML::NodeType::Map);
    ArmConfig arm;
    if (node["name"]) {
        arm.name = readScalar<std::string>(node["name"], context + ".name");
    }
    if (node["type"]) {
        arm.type = readScalar<std::string>(node["type"], context + ".type");
    }
    if (node["joint_size"]) {
        arm.joint_size = readScalar<int>(node["joint_size"], context + ".joint_size");
    }
    if (node["urdf_joints"]) {
        arm.urdf_joints = readStringSequence(node["urdf_joints"], context + ".urdf_joints");
        if (arm.joint_size <= 0) {
            arm.joint_size = static_cast<int>(arm.urdf_joints.size());
        }
    }
    if (node["urdf_links"]) {
        arm.urdf_links = readStringSequence(node["urdf_links"], context + ".urdf_links");
    }
    if (node["enable_joint"]) {
        arm.enable_joint = readBoolSequence(node["enable_joint"], context + ".enable_joint");
    }
    if (node["pid"]) {
        arm.pid = parseArmPidConfig(node["pid"], context + ".pid");
    }
    if (node["safety"]) {
        arm.safety = parseArmSafetyConfig(node["safety"], context + ".safety");
    }
    if (node["force_control"]) {
        arm.force_control = parseArmForceControlConfig(
            node["force_control"], context + ".force_control");
    }
    return arm;
}

GripperConfig parseGripperConfig(
    const YAML::Node& node,
    const std::string& context) {
    requireNode(node, context, YAML::NodeType::Map);
    GripperConfig gripper;
    if (node["joint_size"]) {
        gripper.joint_size = readScalar<int>(node["joint_size"], context + ".joint_size");
    }
    if (node["offset_from_ee"]) {
        readFloatArray3(
            node["offset_from_ee"], context + ".offset_from_ee", gripper.offset_from_ee);
    }
    return gripper;
}

RobotSectionConfig parseRobotSectionConfig(
    const YAML::Node& node,
    const std::string& context) {
    requireNode(node, context, YAML::NodeType::Map);
    RobotSectionConfig robot;
    if (node["name"]) {
        robot.name = readScalar<std::string>(node["name"], context + ".name");
    }
    if (node["urdf"]) {
        robot.urdf = readScalar<std::string>(node["urdf"], context + ".urdf");
    }
    if (node["arm"]) {
        requireNode(node["arm"], context + ".arm", YAML::NodeType::Sequence);
        robot.arms.reserve(node["arm"].size());
        for (std::size_t i = 0; i < node["arm"].size(); ++i) {
            robot.arms.push_back(parseArmConfig(
                node["arm"][i], context + ".arm[" + std::to_string(i) + "]"));
        }
    }
    if (node["gripper"]) {
        requireNode(node["gripper"], context + ".gripper", YAML::NodeType::Sequence);
        robot.grippers.reserve(node["gripper"].size());
        for (std::size_t i = 0; i < node["gripper"].size(); ++i) {
            robot.grippers.push_back(parseGripperConfig(
                node["gripper"][i], context + ".gripper[" + std::to_string(i) + "]"));
        }
    }
    return robot;
}

SelfCollisionDetectionConfig parseSelfCollisionDetectionConfig(
    const YAML::Node& node,
    const std::string& context) {
    requireNode(node, context, YAML::NodeType::Map);
    SelfCollisionDetectionConfig config;
    if (node["enable"]) {
        config.enable = readScalar<bool>(node["enable"], context + ".enable");
    }
    if (node["use_curobo_spheres"]) {
        config.use_curobo_spheres =
            readScalar<bool>(node["use_curobo_spheres"], context + ".use_curobo_spheres");
    }
    if (node["buffer_boundary"]) {
        config.buffer_boundary =
            readScalar<float>(node["buffer_boundary"], context + ".buffer_boundary");
    }
    return config;
}

RtControlConfig parseRtControlConfig(
    const YAML::Node& node,
    const std::string& context) {
    requireNode(node, context, YAML::NodeType::Map);
    RtControlConfig rt_control;
    if (node["dt_control_ms"]) {
        rt_control.dt_control_ms =
            readScalar<float>(node["dt_control_ms"], context + ".dt_control_ms");
    }
    if (node["robotics_library"]) {
        rt_control.robotics_library =
            readScalar<std::string>(node["robotics_library"], context + ".robotics_library");
    }
    if (node["actuator_mode_under_position_control"]) {
        rt_control.actuator_mode_under_position_control = readScalar<std::string>(
            node["actuator_mode_under_position_control"],
            context + ".actuator_mode_under_position_control");
    }
    if (node["self_collision_detection"]) {
        rt_control.self_collision_detection = parseSelfCollisionDetectionConfig(
            node["self_collision_detection"], context + ".self_collision_detection");
    }
    return rt_control;
}

std::optional<std::filesystem::file_time_type> configPathMtime(
    const std::filesystem::path& config_path) {
    std::error_code ec;
    if (!std::filesystem::exists(config_path, ec) || ec) {
        return std::nullopt;
    }
    const auto mtime = std::filesystem::last_write_time(config_path, ec);
    if (ec) {
        return std::nullopt;
    }
    return mtime;
}

}  // namespace

std::string mapYamlArmTypeToInternal(const std::string& yaml_type) {
    if (yaml_type == "Spark2_V2") {
        return "CuarmV2_2";
    }
    return yaml_type;
}

void clearRobotConfigCache() {
    std::lock_guard<std::mutex> lock(g_robot_config_cache_mutex);
    g_robot_config_cache.clear();
}

RobotConfigDocument parseRobotConfigDocument(const YAML::Node& root) {
    if (!root || !root.IsMap()) {
        throw YamlConfigParseError("config root must be a map");
    }

    RobotConfigDocument document;
    if (root["model"]) {
        document.model = readScalar<std::string>(root["model"], "model");
    }
    if (root["robot"]) {
        document.robot = parseRobotSectionConfig(root["robot"], "robot");
    }
    if (root["rt_control"]) {
        document.rt_control = parseRtControlConfig(root["rt_control"], "rt_control");
    }
    if (root["curobo"]) {
        document.curobo_yaml = YAML::Clone(root["curobo"]);
        document.has_curobo_yaml = true;
    }
    return document;
}

RobotConfigDocument loadRobotConfigDocument(const std::string& yaml_path) {
    const YAML::Node root = loadYamlConfig(yaml_path);
    return parseRobotConfigDocument(root);
}

std::optional<RobotConfigDocument> tryLoadRobotConfigFromPrefix(
    const std::string& config_prefix_path) {
    if (config_prefix_path.empty()) {
        return RobotConfigDocument{};
    }

    const std::filesystem::path config_path =
        std::filesystem::path(config_prefix_path) / "config.yaml";
    const std::optional<std::filesystem::file_time_type> mtime = configPathMtime(config_path);
    if (!mtime.has_value()) {
        return std::nullopt;
    }

    std::error_code ec;
    const std::string cache_key =
        std::filesystem::absolute(config_path, ec).lexically_normal().string();
    {
        std::lock_guard<std::mutex> lock(g_robot_config_cache_mutex);
        const auto it = g_robot_config_cache.find(cache_key);
        if (it != g_robot_config_cache.end() && it->second.mtime == *mtime) {
            return it->second.document;
        }
    }

    RobotConfigDocument document = loadRobotConfigDocument(config_path.string());
    {
        std::lock_guard<std::mutex> lock(g_robot_config_cache_mutex);
        g_robot_config_cache[cache_key] = RobotConfigCacheEntry{*mtime, document};
    }
    return document;
}

const RobotSectionConfig& requireRobotSection(const RobotConfigDocument& document) {
    return requireField(document.robot, "robot");
}

const ArmConfig& requireArm(const RobotConfigDocument& document, int arm_index) {
    const RobotSectionConfig& robot = requireRobotSection(document);
    if (arm_index < 0 || arm_index >= static_cast<int>(robot.arms.size())) {
        throw YamlConfigParseError(
            "robot.arm[" + std::to_string(arm_index) + "] out of range");
    }
    return robot.arms[static_cast<std::size_t>(arm_index)];
}

float controlDtSeconds(const RobotConfigDocument& document) {
    const RtControlConfig& rt_control = requireField(document.rt_control, "rt_control");
    return rt_control.dt_control_ms / 1000.0f;
}

const std::string& roboticsLibraryName(const RobotConfigDocument& document) {
    const RtControlConfig& rt_control = requireField(document.rt_control, "rt_control");
    if (rt_control.robotics_library.empty()) {
        throw YamlConfigParseError("rt_control.robotics_library is empty");
    }
    return rt_control.robotics_library;
}

}  // namespace robot::platform
