#include <iostream>
#include <string>

#include "config_loader.h"

using namespace robot::platform;

int main() {
    std::string project_root_path = PROJECT_ROOT_DIR;
    std::string config_path = project_root_path + "/configuration/config.yaml";
    std::cout << "Loading config from: " << config_path << std::endl;

    try {
        YAML::Node config = loadYamlConfig(config_path);
        printYamlNode(config);
        writeYamlNode(config, "full_config.yaml");
    } catch (const YamlLoadError& e) {
        std::cerr << "Failed to load config: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
