#include "config_loader.h"

#include "robot_config.h"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace robot::platform {
namespace {

using FileTime = std::filesystem::file_time_type;

struct FileYamlCacheEntry {
    FileTime mtime{};
    YAML::Node node;
};

struct ProcessedYamlCacheEntry {
    std::vector<std::pair<std::string, FileTime>> dependencies;
    YAML::Node node;
};

std::mutex g_yaml_cache_mutex;
std::unordered_map<std::string, FileYamlCacheEntry> g_file_yaml_cache;
std::unordered_map<std::string, ProcessedYamlCacheEntry> g_processed_yaml_cache;

std::string cacheKeyForPath(const std::filesystem::path& path) {
    std::error_code ec;
    const std::filesystem::path absolute = std::filesystem::absolute(path, ec);
    if (ec) {
        return path.lexically_normal().string();
    }
    return absolute.lexically_normal().string();
}

std::optional<FileTime> queryFileMtime(const std::filesystem::path& path) {
    std::error_code ec;
    if (!std::filesystem::exists(path, ec) || ec) {
        return std::nullopt;
    }
    const FileTime mtime = std::filesystem::last_write_time(path, ec);
    if (ec) {
        return std::nullopt;
    }
    return mtime;
}

bool processedCacheStillValid(const ProcessedYamlCacheEntry& entry) {
    for (const auto& dep : entry.dependencies) {
        const std::optional<FileTime> current = queryFileMtime(dep.first);
        if (!current.has_value() || *current != dep.second) {
            return false;
        }
    }
    return true;
}

void mergeYamlNodes(YAML::Node& target, const YAML::Node& source) {
    if (!source.IsMap()) {
        target = source;
        return;
    }
    if (!target || !target.IsMap()) {
        target = YAML::Node(YAML::NodeType::Map);
    }

    for (auto it = source.begin(); it != source.end(); ++it) {
        const std::string key = it->first.as<std::string>();
        const YAML::Node& value = it->second;
        YAML::Node existing = target[key];
        if (existing && existing.IsMap() && value.IsMap()) {
            mergeYamlNodes(existing, value);
            target[key] = existing;
        } else if (existing && existing.IsSequence() && value.IsSequence()) {
            for (std::size_t i = 0; i < value.size(); ++i) {
                target[key].push_back(value[i]);
            }
        } else {
            target[key] = value;
        }
    }
}

YAML::Node loadYamlStream(std::istream& stream, const std::string& context_path) {
    try {
        return YAML::Load(stream);
    } catch (const YAML::ParserException& e) {
        throw YamlLoadError(
            "YAML syntax error in '" + context_path + "': " + e.what());
    } catch (const YAML::Exception& e) {
        throw YamlLoadError(
            "YAML error in '" + context_path + "': " + e.what());
    }
}

YAML::Node loadYamlFileUncached(const std::filesystem::path& path) {
    const std::string context_path = path.string();
    std::ifstream stream(path);
    if (!stream.is_open()) {
        throw YamlLoadError("Failed to open YAML file: " + context_path);
    }
    YAML::Node node = loadYamlStream(stream, context_path);
    stream.close();
    return node;
}

YAML::Node loadYamlFileCached(const std::filesystem::path& path) {
    const std::optional<FileTime> mtime = queryFileMtime(path);
    if (!mtime.has_value()) {
        throw YamlLoadError("YAML file not found: " + path.string());
    }

    const std::string cache_key = cacheKeyForPath(path);
    {
        std::lock_guard<std::mutex> lock(g_yaml_cache_mutex);
        const auto it = g_file_yaml_cache.find(cache_key);
        if (it != g_file_yaml_cache.end() && it->second.mtime == *mtime) {
            return YAML::Clone(it->second.node);
        }
    }

    YAML::Node loaded = loadYamlFileUncached(path);
    {
        std::lock_guard<std::mutex> lock(g_yaml_cache_mutex);
        g_file_yaml_cache[cache_key] = FileYamlCacheEntry{*mtime, YAML::Clone(loaded)};
    }
    return loaded;
}

YAML::Node processImportsRecursive(
    const YAML::Node& config,
    const std::string& base_dir,
    std::unordered_set<std::string>& visited,
    std::vector<std::pair<std::string, FileTime>>& dependencies) {
    auto processSingleImport = [&](const std::string& import_path) -> YAML::Node {
        std::filesystem::path full_path;
        if (!import_path.empty() && import_path.front() == '/') {
            full_path = std::filesystem::path(import_path);
        } else {
            full_path = std::filesystem::path(base_dir) / import_path;
        }
        full_path = full_path.lexically_normal();

        const std::string visit_key = cacheKeyForPath(full_path);
        if (!visited.insert(visit_key).second) {
            throw YamlLoadError("Circular import_yaml detected: " + visit_key);
        }

        const std::optional<FileTime> mtime = queryFileMtime(full_path);
        if (!mtime.has_value()) {
            throw YamlLoadError("Failed to open imported YAML file: " + full_path.string());
        }
        dependencies.emplace_back(visit_key, *mtime);

        YAML::Node imported_config = loadYamlFileCached(full_path);
        const std::string import_base = full_path.parent_path().string();
        YAML::Node processed = processImportsRecursive(
            imported_config, import_base, visited, dependencies);
        visited.erase(visit_key);
        return processed;
    };

    if (config.IsMap()) {
        YAML::Node result(YAML::NodeType::Map);
        for (auto it = config.begin(); it != config.end(); ++it) {
            const std::string key = it->first.as<std::string>();
            const YAML::Node& child = it->second;
            if (key == "import_yaml") {
                const YAML::Node imported = processSingleImport(child.as<std::string>());
                mergeYamlNodes(result, imported);
            } else {
                result[key] = processImportsRecursive(child, base_dir, visited, dependencies);
            }
        }
        return result;
    }

    if (config.IsSequence()) {
        YAML::Node result(YAML::NodeType::Sequence);
        for (std::size_t i = 0; i < config.size(); ++i) {
            result.push_back(processImportsRecursive(config[i], base_dir, visited, dependencies));
        }
        return result;
    }

    return config;
}

bool isSimpleSequence(const YAML::Node& seq) {
    for (std::size_t i = 0; i < seq.size(); ++i) {
        if (!seq[i].IsScalar()) {
            return false;
        }
    }
    return true;
}

bool isSequenceOfSimpleSequences(const YAML::Node& seq) {
    if (seq.size() == 0) {
        return false;
    }
    for (std::size_t i = 0; i < seq.size(); ++i) {
        if (!seq[i].IsSequence() || !isSimpleSequence(seq[i])) {
            return false;
        }
    }
    return true;
}

void printSequenceCompact(const YAML::Node& seq) {
    std::cout << "[";
    for (std::size_t i = 0; i < seq.size(); ++i) {
        if (i > 0) {
            std::cout << ", ";
        }
        if (seq[i].IsScalar()) {
            std::cout << seq[i].as<std::string>();
        } else if (seq[i].IsSequence() && isSimpleSequence(seq[i])) {
            printSequenceCompact(seq[i]);
        } else {
            std::cout << "...";
        }
    }
    std::cout << "]";
}

void print2DArrayCompact(const YAML::Node& seq) {
    std::cout << "[";
    for (std::size_t i = 0; i < seq.size(); ++i) {
        if (i > 0) {
            std::cout << ", ";
        }
        printSequenceCompact(seq[i]);
    }
    std::cout << "]";
}

void printYamlNode(const YAML::Node& node, int indent, const std::string& first_line_prefix) {
    const std::string prefix(static_cast<std::size_t>(indent) * 2, ' ');
    const std::string line_prefix = first_line_prefix.empty() ? prefix : first_line_prefix;

    if (node.IsScalar()) {
        std::cout << node.as<std::string>() << std::endl;
        return;
    }

    if (node.IsMap()) {
        for (auto it = node.begin(); it != node.end(); ++it) {
            const std::string key = it->first.as<std::string>();
            const YAML::Node value = node[key];
            const std::string use_prefix = (it == node.begin()) ? line_prefix : prefix;
            std::cout << use_prefix << key << ": ";
            if (value.IsScalar()) {
                std::cout << value.as<std::string>() << std::endl;
            } else if (value.IsSequence() && isSimpleSequence(value)) {
                printSequenceCompact(value);
                std::cout << std::endl;
            } else if (value.IsSequence() && isSequenceOfSimpleSequences(value)) {
                print2DArrayCompact(value);
                std::cout << std::endl;
            } else if (value.IsSequence()) {
                std::cout << std::endl;
                for (std::size_t i = 0; i < value.size(); ++i) {
                    if (value[i].IsScalar()) {
                        std::cout << prefix << "  - " << value[i].as<std::string>() << std::endl;
                    } else {
                        printYamlNode(value[i], indent + 2, prefix + "  - ");
                    }
                }
            } else if (value.IsMap()) {
                std::cout << std::endl;
                printYamlNode(value, indent + 1, "");
            } else {
                std::cout << std::endl;
            }
        }
        return;
    }

    if (node.IsSequence()) {
        for (std::size_t i = 0; i < node.size(); ++i) {
            if (node[i].IsMap()) {
                printYamlNode(node[i], indent + 1, prefix + "- ");
            } else {
                std::cout << prefix << "- ";
                if (node[i].IsScalar()) {
                    std::cout << node[i].as<std::string>() << std::endl;
                } else if (node[i].IsSequence()) {
                    printSequenceCompact(node[i]);
                    std::cout << std::endl;
                } else {
                    std::cout << std::endl;
                }
            }
        }
    }
}

}  // namespace

void clearYamlCache() {
    std::lock_guard<std::mutex> lock(g_yaml_cache_mutex);
    g_file_yaml_cache.clear();
    g_processed_yaml_cache.clear();
    clearRobotConfigCache();
}

YAML::Node loadYamlFile(const std::string& filename) {
    return loadYamlFileCached(std::filesystem::path(filename));
}

YAML::Node loadYamlConfig(const std::string& filename) {
    const std::filesystem::path path(filename);
    const std::string cache_key = cacheKeyForPath(path);
    const std::optional<FileTime> root_mtime = queryFileMtime(path);
    if (!root_mtime.has_value()) {
        throw YamlLoadError("YAML file not found: " + path.string());
    }

    {
        std::lock_guard<std::mutex> lock(g_yaml_cache_mutex);
        const auto it = g_processed_yaml_cache.find(cache_key);
        if (it != g_processed_yaml_cache.end() && processedCacheStillValid(it->second)) {
            return YAML::Clone(it->second.node);
        }
    }

    YAML::Node root_config = loadYamlFileCached(path);
    std::unordered_set<std::string> visited;
    std::vector<std::pair<std::string, FileTime>> dependencies;
    dependencies.emplace_back(cache_key, *root_mtime);
    visited.insert(cache_key);

    YAML::Node processed = processImportsRecursive(
        root_config,
        path.parent_path().string(),
        visited,
        dependencies);

    {
        std::lock_guard<std::mutex> lock(g_yaml_cache_mutex);
        g_processed_yaml_cache[cache_key] =
            ProcessedYamlCacheEntry{dependencies, YAML::Clone(processed)};
    }
    return processed;
}

bool tryLoadYamlConfig(const std::string& filename, YAML::Node& out, std::string& error_message) {
    try {
        out = loadYamlConfig(filename);
        return true;
    } catch (const std::exception& e) {
        error_message = e.what();
        out.reset();
        return false;
    }
}

void printYamlNode(const YAML::Node& node) {
    printYamlNode(node, 0, "");
}

bool writeYamlNode(const YAML::Node& node, const std::string& write_path) {
    try {
        std::ofstream fout(write_path);
        if (!fout.is_open()) {
            return false;
        }
        fout << node;
        fout.close();
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

}  // namespace robot::platform
