#pragma once

#include "controller/config.hpp"

#include <string>

namespace runtime
{

struct logger_config
{
    bool stdout_block = false;
    float hz = 5.0f;
};

struct app_config
{
    std::string ipc_prefix = "wbr";
    controller::chassis_config chassis{};
    controller::imu_sim_config imu_sim{};
    controller::imu_mode imu_mode = controller::imu_mode::mahony;
    float control_hz = 1000.0f;
    logger_config logger{};
};

app_config load_config(int argc, char** argv);

}  // namespace runtime
