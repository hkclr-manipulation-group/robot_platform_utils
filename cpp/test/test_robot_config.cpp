#include <cassert>
#include <iostream>
#include <string>

#include "config_loader.h"
#include "robot_config.h"

using namespace robot::platform;

int main() {
    std::string project_root_path = PROJECT_ROOT_DIR;
    const std::string config_path = project_root_path + "/configuration/config.yaml";
    std::cout << "Loading robot config from: " << config_path << std::endl;

    clearYamlCache();

    RobotConfigDocument first = loadRobotConfigDocument(config_path);
    assert(first.model == "spark2_v2");
    assert(first.robot.has_value());
    assert(first.robot->name == "arm_v2_no_gripper");
    assert(first.robot->urdf == "arm_v2.urdf");
    assert(first.robot->arms.size() == 1);
    assert(first.robot->arms[0].joint_size == 6);
    assert(first.robot->arms[0].type == "Spark2_V2");
    assert(mapYamlArmTypeToInternal(first.robot->arms[0].type) == "CuarmV2_2");
    assert(first.robot->arms[0].safety.has_value());
    assert(first.robot->arms[0].safety->joint_soft_limit.has_value());
    assert(first.robot->arms[0].safety->joint_soft_limit->has_velocity);
    assert(first.robot->arms[0].safety->joint_soft_limit->velocity[0] == 300.f);

    RobotConfigDocument cached = loadRobotConfigDocument(config_path);
    assert(cached.model == first.model);

    const std::optional<RobotConfigDocument> from_prefix =
        tryLoadRobotConfigFromPrefix(project_root_path + "/configuration");
    assert(from_prefix.has_value());
    assert(from_prefix->model == "spark2_v2");

    const std::optional<RobotConfigDocument> missing =
        tryLoadRobotConfigFromPrefix(project_root_path + "/configuration/missing_dir");
    assert(!missing.has_value());

    std::cout << "test_robot_config passed" << std::endl;
    return 0;
}
