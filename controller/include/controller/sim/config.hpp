#pragma once

#include <array>
#include <cstdint>
#include <string>

#include "control/config.hpp"
#include "control/imu_fusion.hpp"

namespace controller::sim
{

inline constexpr float kControlDtSeconds = 0.001f;

struct ImuStateInterfaces
{
    std::string quaternion;
    std::string angular_velocity;
    std::string linear_acceleration;
};

struct MotorInterfaces
{
    std::string semantic_name;
    std::string position_state;
    std::string velocity_state;
    std::string effort_state;
    std::string effort_command;
};

struct InterfaceConfig
{
    ImuStateInterfaces imu;
    std::array<MotorInterfaces, 6> motors;
};

InterfaceConfig MakeDefaultInterfaceConfig();

struct PipelineConfig
{
    control::chassis_config chassis{};
    control::imu_mode imu_mode = control::imu_mode::mahony;
    control::mahony_config mahony{};
};

struct AdapterConfig
{
    std::string server_name = "wbr";
    std::string controller_id = "wbr_ctrl";
    std::uint32_t timeout_ms = 5000;
    std::uint32_t service_timeout_ms = 10000;
    std::uint64_t max_ticks = 0;
    std::uint32_t operator_input_max_age_ms = 200;
    InterfaceConfig interfaces = MakeDefaultInterfaceConfig();
};

}  // namespace controller::sim
