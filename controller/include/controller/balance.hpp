#pragma once

#include "controller/config.hpp"
#include "controller/lqr_coeffs.hpp"
#include "controller/math.hpp"
#include "controller/msgs.hpp"
#include "controller/leg.hpp"

#include <cmath>
#include <cstring>

namespace controller
{

class lqr_solver
{
public:
    float tout[4] = {};

    lqr_mode mode = lqr_mode::low;

    void update(float llen, float rlen, const float ref_x[10], const float obs_x[10], const chassis_config& cfg);
};

class odometry
{
public:
    float x = 0.0f;
    float v = 0.0f;
    float az = 0.0f;

    void reset();

    void update(const float quaternion[4], const float acc[3], float vel_meas, float yaw, float dt);

private:
    static constexpr float k_max_forward_accel = 12.0f;
    static constexpr float k_max_velocity_innovation = 1.5f;
    static constexpr float k_max_accel_innovation = 15.0f;

    float x_hat_[3] = {};
    float p_[9] = {10, 0, 0, 0, 10, 0, 0, 0, 10};
    float p_init_[9] = {10, 0, 0, 0, 10, 0, 0, 0, 10};
    float q_[9] = {0.00025f, 0.00125f, 0.005f, 0.00125f, 0.005f, 0.05f, 0.005f, 0.05f, 0.5f};
    float r_[4] = {0.1f, 0.0f, 0.0f, 50.0f};
};

class command_fusion
{
public:
    void reset(const chassis_config& cfg);

    void update(const input_snapshot_t& input, const msg_pendulum_t& pendulum, const msg_ins_t& ins,
                const chassis_config& cfg, float dt);

    const msg_cmd_t& msg() const;

private:
    msg_cmd_t msg_{};
    bool move_enabled_ = false;
    bool space_prev_ = false;
    bool space_armed_ = false;
    bool yaw_ref_initialized_ = false;
    bool yaw_active_prev_ = false;
    slope yaw_slope_{0.0f, 0.006f};
    float len_target_ = 0.16f;
};

}  // namespace controller
