#ifndef CONFIG_LOADER_H
#define CONFIG_LOADER_H

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <string>

namespace robot::platform {

class YamlLoadError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

/** Clear all in-memory YAML caches (mainly for tests). */
void clearYamlCache();

/** Load a single YAML file (no `import_yaml` expansion). Throws YamlLoadError. Cached by path + mtime. */
YAML::Node loadYamlFile(const std::string& filename);

/**
 * Load YAML and expand `import_yaml` directives recursively.
 * Throws YamlLoadError on I/O or syntax errors. Cached by root path + dependency mtimes.
 */
YAML::Node loadYamlConfig(const std::string& filename);

/** Non-throwing variant of loadYamlConfig. */
bool tryLoadYamlConfig(const std::string& filename, YAML::Node& out, std::string& error_message);

void printYamlNode(const YAML::Node& node);
bool writeYamlNode(const YAML::Node& node, const std::string& write_path);

}  // namespace robot::platform

#endif
