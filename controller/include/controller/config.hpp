#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "control/imu_fusion.hpp"
#include "controller/sim/config.hpp"
#include "controller/sim/input_script.hpp"

namespace controller
{

struct logger_config
{
    bool stdout_block = false;
    float hz = 5.0f;
};

struct app_config
{
    std::string ipc_prefix = "wbr";
    sim::AdapterConfig adapter{};
    sim::PipelineConfig pipeline{};
    sim::InputScript input_script{};
    control::imu_sim_config imu_sim{};
    logger_config logger{};
    std::uint64_t log_every_ticks = 0;
};

app_config load_config(int argc, char** argv);

}  // namespace controller
