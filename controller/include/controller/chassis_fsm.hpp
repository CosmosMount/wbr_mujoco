#pragma once

#include "controller/balance.hpp"
#include "controller/config.hpp"
#include "controller/leg.hpp"
#include "controller/math.hpp"
#include "controller/msgs.hpp"

#include <cmath>
#include <cstdint>

namespace controller
{

struct fsm_inputs
{
    const msg_ins_t& ins;
    const msg_cmd_t& cmd;
    const msg_odometry_t& odom;
    leg_controller& left;
    leg_controller& right;
    float observed_x[10] = {};
    float n_total = 0.0f;
    bool chassis_dead = false;
};

struct fsm_outputs
{
    msg_ctrl_t ctrl{};
    msg_pendulum_t pendulum{};
    msg_motor_cmd_t motor{};
    chassis_state next_state{};
};

class chassis_fsm
{
public:
    void init(const chassis_config& cfg);

    void reset();

    chassis_state state() const;

    void step(const fsm_inputs& in, fsm_outputs& out);

private:
    void step_relax(const fsm_inputs& in, fsm_outputs& out, float pitch, float fl[2], float fr[2], float& twl,
                    float& twr);

    void step_recover(const fsm_inputs& in, float pitch, float fl[2], float fr[2], float& twl, float& twr);

    void step_flatten(const fsm_inputs& in, fsm_outputs& out, float fl[2], float fr[2], float& twl, float& twr);

    void step_neutral(const fsm_inputs& in, float fl[2], float fr[2], float& twl, float& twr);

    void step_normal(const fsm_inputs& in, fsm_outputs& out, float pitch, float fl[2], float fr[2], float& twl,
                     float& twr);

    void step_offground(const fsm_inputs& in, float pitch, float fl[2], float fr[2], float& twl, float& twr);

    void step_spin(const fsm_inputs& in, float fl[2], float fr[2], float& twl, float& twr);

    chassis_config cfg_ = k_default_chassis;
    chassis_state state_ = chassis_state::relax;
    lqr_solver lqr_;
    pid roll_pd_{k_default_chassis.fsm_pid.roll};
    slope len_slope_{0.16f, 0.0002f};
    float ref_x_[10] = {};
    float target_len_ = 0.16f;
    std::uint32_t air_protect_cnt_ = 0;
    std::uint32_t flipover_cnt_ = 0;
    std::uint32_t landing_cnt_ = 0;
    std::uint32_t normal_enter_cnt_ = 0;
    std::uint32_t normal_exit_cnt_ = 0;
    bool flying_ = false;
    bool going_stair_ = false;
};

}  // namespace controller
