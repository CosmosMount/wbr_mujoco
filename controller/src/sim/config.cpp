#include "controller/sim/config.hpp"

namespace controller::sim
{

InterfaceConfig MakeDefaultInterfaceConfig()
{
    return {
        {"imu_quat", "imu_gyro", "imu_accel"},
        {{{"left_joint_1", "ljoint1_pos", "ljoint1_vel", "ljoint1_tor", "ljoint1_actuator"},
          {"left_joint_4", "ljoint4_pos", "ljoint4_vel", "ljoint4_tor", "ljoint4_actuator"},
          {"left_wheel", "lwheel_pos", "lwheel_vel", "lwheel_tor", "lwheel_actuator"},
          {"right_joint_1", "rjoint1_pos", "rjoint1_vel", "rjoint1_tor", "rjoint1_actuator"},
          {"right_joint_4", "rjoint4_pos", "rjoint4_vel", "rjoint4_tor", "rjoint4_actuator"},
          {"right_wheel", "rwheel_pos", "rwheel_vel", "rwheel_tor", "rwheel_actuator"}}},
    };
}

}  // namespace controller::sim
