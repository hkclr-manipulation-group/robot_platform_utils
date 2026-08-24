#include <pybind11/pybind11.h>
#include <yaml-cpp/yaml.h>

#include "config_loader.h"
#include "robot_config.h"

namespace py = pybind11;

// Recursive converter: YAML::Node → Python object
py::object to_python(const YAML::Node& node) {
    if (node.IsScalar()) {        try {
            return py::int_(node.as<int>());
        } catch (const YAML::BadConversion&) {
            try {
                return py::float_(node.as<double>());
            } catch (const YAML::BadConversion&) {
                try {
                    return py::bool_(node.as<bool>());
                } catch (const YAML::BadConversion&) {
                    return py::str(node.as<std::string>());
                }
            }
        }

    } else if (node.IsSequence()) {
        py::list lst;
        for (auto it : node) {
            lst.append(to_python(it));
        }
        return lst;
    } else if (node.IsMap()) {
        py::dict d;
        for (auto it : node) {
            d[py::str(it.first.as<std::string>())] = to_python(it.second);
        }
        return d;
    }
    return py::none();
}

PYBIND11_MODULE(config_loader, m){
    m.doc() = "Bindings for YAML config loader";

    py::register_exception<robot::platform::YamlLoadError>(m, "YamlLoadError");
    py::register_exception<robot::platform::YamlConfigParseError>(m, "YamlConfigParseError");

    py::class_<YAML::Node>(m, "YamlNode")
        .def("print", &printYamlNode)
        .def("as_dict", [](const YAML::Node& n) {
            return to_python(n);
        });

    m.def("load_yaml", &loadYamlConfig, py::arg("filename"), "Load a YAML file");
    m.def("try_load_yaml", [](const std::string& filename) {
        YAML::Node node;
        std::string error;
        if (!tryLoadYamlConfig(filename, node, error)) {
            throw robot::platform::YamlLoadError(error);
        }
        return node;
    }, py::arg("filename"), "Load a YAML file without raising on failure path");
    m.def(
        "load_robot_config",
        &robot::platform::loadRobotConfigDocument,
        py::arg("filename"),
        "Load and parse a robot config.yaml into a typed document");
    m.def(
        "load_robot_config_from_prefix",
        [](const std::string& prefix) -> py::object {
            const auto document = robot::platform::tryLoadRobotConfigFromPrefix(prefix);
            if (!document.has_value()) {
                return py::none();
            }
            py::dict result;
            if (!document->model.empty()) {
                result["model"] = document->model;
            }
            if (document->robot.has_value()) {
                py::dict robot;
                if (!document->robot->name.empty()) {
                    robot["name"] = document->robot->name;
                }
                if (!document->robot->urdf.empty()) {
                    robot["urdf"] = document->robot->urdf;
                }
                if (!document->robot->arms.empty()) {
                    py::list arms;
                    for (const auto& arm : document->robot->arms) {
                        py::dict arm_dict;
                        arm_dict["name"] = arm.name;
                        arm_dict["type"] = arm.type;
                        arm_dict["joint_size"] = arm.joint_size;
                        arms.append(arm_dict);
                    }
                    robot["arm"] = arms;
                }
                result["robot"] = robot;
            }
            return result;
        },
        py::arg("config_prefix_path"),
        "Load typed robot config from `<prefix>/config.yaml`; returns None if missing");
    m.def("clear_yaml_cache", &clearYamlCache, "Clear YAML and robot-config caches");
}

