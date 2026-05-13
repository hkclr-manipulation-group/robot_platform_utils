#include <iostream>
#include "config_loader.h"

int main() {
    YAML::Node config = loadYamlConfig("/mnt/ssd/user/ruby/cuarm_panel_control/cuarm_configuration/dual_v2_1/config.yaml");
    printYamlNode(config);
    // printYamlNode(config["robot"][0]["gripper"]);
    writeYamlNode(config, "full_config.yaml");
    return 0;
}

