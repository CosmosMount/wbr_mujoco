// Model contract test: verify WBR MJCF exposes the interface resource names from wbr.yaml.
// Run from wbr_mujoco repo root:
//   ./build/test_import
//   ./build/test_import config/robots/wbr.yaml

#include "controller/sim/config.hpp"

#include <mujoco/mujoco.h>

#include <cstdio>
#include <cstring>
#include <string>
#include <unordered_set>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace
{

std::string resolve_path(const std::string& config_path, const std::string& relative_path)
{
    const auto slash = config_path.find_last_of('/');
    if (slash == std::string::npos)
    {
        return relative_path;
    }
    return config_path.substr(0, slash + 1) + relative_path;
}

bool load_scene_path(const std::string& config_path, std::string& scene_path, std::string& error)
{
    try
    {
        const YAML::Node root = YAML::LoadFile(config_path);
        if (!root["scene"])
        {
            error = "missing scene in config";
            return false;
        }
        scene_path = resolve_path(config_path, root["scene"].as<std::string>());
        return true;
    }
    catch (const std::exception& ex)
    {
        error = ex.what();
        return false;
    }
}

bool sensor_exists(mjModel* model, const char* name)
{
    return mj_name2id(model, mjOBJ_SENSOR, name) >= 0;
}

bool actuator_exists(mjModel* model, const char* name)
{
    return mj_name2id(model, mjOBJ_ACTUATOR, name) >= 0;
}

}  // namespace

int main(int argc, char** argv)
{
    const std::string config_path = (argc > 1) ? argv[1] : "config/robots/wbr.yaml";
    std::string error;

    std::string scene_path;
    if (!load_scene_path(config_path, scene_path, error))
    {
        std::fprintf(stderr, "load scene failed: %s\n", error.c_str());
        return 1;
    }

    char load_error[1024] = {};
    mjModel* model = mj_loadXML(scene_path.c_str(), nullptr, load_error, sizeof(load_error));
    if (!model)
    {
        std::fprintf(stderr, "mj_loadXML(%s) failed: %s\n", scene_path.c_str(), load_error);
        return 1;
    }

    const controller::sim::InterfaceConfig interfaces = controller::sim::MakeDefaultInterfaceConfig();
    std::vector<std::string> observations = {
        interfaces.imu.quaternion,
        interfaces.imu.linear_acceleration,
        interfaces.imu.angular_velocity,
    };
    for (const auto& motor : interfaces.motors)
    {
        observations.push_back(motor.position_state);
    }
    for (const auto& motor : interfaces.motors)
    {
        observations.push_back(motor.velocity_state);
    }
    for (const auto& motor : interfaces.motors)
    {
        observations.push_back(motor.effort_state);
    }
    std::vector<std::string> commands;
    for (const auto& motor : interfaces.motors)
    {
        commands.push_back(motor.effort_command);
    }

    for (const auto& name : observations)
    {
        if (!sensor_exists(model, name.c_str()))
        {
            std::fprintf(stderr, "missing observation sensor: %s\n", name.c_str());
            mj_deleteModel(model);
            return 1;
        }
    }

    for (const auto& name : commands)
    {
        if (!actuator_exists(model, name.c_str()))
        {
            std::fprintf(stderr, "missing command actuator: %s\n", name.c_str());
            mj_deleteModel(model);
            return 1;
        }
    }

    const int home_id = mj_name2id(model, mjOBJ_KEY, "home");
    if (home_id < 0)
    {
        std::fprintf(stderr, "missing home keyframe\n");
        mj_deleteModel(model);
        return 1;
    }

    std::printf("test_import ok\n");
    std::printf("  config: %s\n", config_path.c_str());
    std::printf("  scene:  %s\n", scene_path.c_str());
    std::printf("  observations: %zu\n", observations.size());
    std::printf("  commands: %zu\n", commands.size());
    std::printf("  timestep: %.6f\n", model->opt.timestep);

    mj_deleteModel(model);
    return 0;
}
