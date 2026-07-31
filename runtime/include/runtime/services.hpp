#pragma once

#include "runtime/config.hpp"
#include "controller/msgs.hpp"
#include "msg/msg.hpp"

#include <atomic>
#include <thread>

namespace runtime
{

class ecal_io;

class actuator_service
{
public:
    actuator_service(const app_config& cfg, ecal_io& io, std::atomic<bool>& running);
    ~actuator_service();

private:
    void loop();

    const app_config& cfg_;
    ecal_io& io_;
    std::atomic<bool>& running_;
    msg::subscriber sub_reset_ = msg::subscribe<controller::sim_reset_t>();
    msg::subscriber sub_motor_cmd_ = msg::subscribe<controller::msg_motor_cmd_t>();
    controller::msg_motor_cmd_t motor_cmd_{};
    std::thread thread_;
};

class ins_service
{
public:
    ins_service(const app_config& cfg, ecal_io& io, std::atomic<bool>& running);
    ~ins_service();

private:
    void loop();

    const app_config& cfg_;
    ecal_io& io_;
    std::atomic<bool>& running_;
    msg::subscriber sub_reset_ = msg::subscribe<controller::sim_reset_t>();
    msg::subscriber sub_raw_state_ = msg::subscribe<controller::msg_raw_state_t>();
    std::thread thread_;
};

class command_service
{
public:
    command_service(const app_config& cfg, std::atomic<bool>& running);
    ~command_service();

private:
    void loop();

    const app_config& cfg_;
    std::atomic<bool>& running_;
    msg::subscriber sub_pendulum_ = msg::subscribe<controller::msg_pendulum_t>();
    msg::subscriber sub_ins_ = msg::subscribe<controller::msg_ins_t>();
    msg::subscriber sub_reset_ = msg::subscribe<controller::sim_reset_t>();
    msg::subscriber sub_input_ = msg::subscribe<controller::input_snapshot_t>();
    controller::input_snapshot_t input_{};
    controller::msg_pendulum_t pendulum_{};
    controller::msg_ins_t ins_{};
    std::thread thread_;
};

class chassis_service
{
public:
    chassis_service(const app_config& cfg, std::atomic<bool>& running);
    ~chassis_service();

private:
    void loop();

    const app_config& cfg_;
    std::atomic<bool>& running_;
    msg::subscriber sub_reset_ = msg::subscribe<controller::sim_reset_t>();
    msg::subscriber sub_raw_state_ = msg::subscribe<controller::msg_raw_state_t>();
    msg::subscriber sub_ins_ = msg::subscribe<controller::msg_ins_t>();
    msg::subscriber sub_cmd_ = msg::subscribe<controller::msg_cmd_t>();
    std::thread thread_;
};

class sim_log_service
{
public:
    sim_log_service(const app_config& cfg, std::atomic<bool>& running);
    ~sim_log_service();

private:
    void loop();

    const app_config& cfg_;
    std::atomic<bool>& running_;
    msg::subscriber sub_log_ = msg::subscribe<controller::msg_log_t>();
    std::thread thread_;
};

class web_visualizer_service
{
public:
    web_visualizer_service(const app_config& cfg, std::atomic<bool>& running);
    ~web_visualizer_service();

private:
    void loop();

    const app_config& cfg_;
    std::atomic<bool>& running_;
    msg::subscriber sub_log_ = msg::subscribe<controller::msg_log_t>();
    std::thread thread_;
};

}  // namespace runtime
