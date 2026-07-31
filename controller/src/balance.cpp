#include "controller/balance.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace controller
{

void lqr_solver::update(float llen, float rlen, const float ref_x[10], const float obs_x[10], const chassis_config& cfg)
{
        llen = clamp(llen, cfg.lmin, cfg.lmax);
        rlen = clamp(rlen, cfg.lmin, cfg.lmax);
        llen = std::round(llen * 100.0f) / 100.0f;
        rlen = std::round(rlen * 100.0f) / 100.0f;

        const float (*table)[6] = k_lqr_low;
        if (mode == lqr_mode::high)
        {
            table = k_lqr_high;
        }
        else if (mode == lqr_mode::spin)
        {
            table = k_lqr_spin;
        }

        float kbuf[40] = {};
        for (int i = 0; i < 40; ++i)
        {
            kbuf[i] = table[i][0] + table[i][1] * llen + table[i][2] * rlen + table[i][3] * llen * llen +
                      table[i][4] * llen * rlen + table[i][5] * rlen * rlen;
        }

        float err[10] = {};
        for (int i = 0; i < 10; ++i)
        {
            err[i] = obs_x[i] - ref_x[i];
        }

        for (int i = 0; i < 4; ++i)
        {
            float temp = 0.0f;
            for (int j = 0; j < 10; ++j)
            {
                temp += kbuf[i * 10 + j] * err[j];
            }
            tout[i] = temp;
        }

        tout[0] = clamp(tout[0], -cfg.twheel_max, cfg.twheel_max);
        tout[1] = clamp(tout[1], -cfg.twheel_max, cfg.twheel_max);
        tout[2] = clamp(tout[2], -cfg.thip_max, cfg.thip_max);
        tout[3] = clamp(tout[3], -cfg.thip_max, cfg.thip_max);
    }

void odometry::reset()
{
        x = 0.0f;
        v = 0.0f;
        az = 0.0f;
        std::memset(x_hat_, 0, sizeof(x_hat_));
        std::memcpy(p_, p_init_, sizeof(p_));
    }

void odometry::update(const float quaternion[4], const float acc[3], float vel_meas, float yaw, float dt)
{
        float a_body[3] = {acc[0], acc[1], acc[2]};
        float a_world[3] = {};
        quat_rotate_vec(quaternion, a_body, a_world);
        const float a_x_raw = a_world[0] * std::cos(yaw) + a_world[1] * std::sin(yaw);
        const float a_x = clamp(a_x_raw, -k_max_forward_accel, k_max_forward_accel);

        az = a_world[2];

        const float z[2] = {vel_meas, a_x};
        const float dt2 = dt * dt;
        const float dt3 = dt2 * dt;

        float f[9] = {1.0f, dt, 0.5f * dt2, 0.0f, 1.0f, dt, 0.0f, 0.0f, 1.0f};
        float x_minus[3] = {};
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                x_minus[i] += f[i * 3 + j] * x_hat_[j];
            }
        }

        float p_minus[9] = {};
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                for (int k = 0; k < 3; ++k)
                {
                    p_minus[i * 3 + j] += f[i * 3 + k] * p_[k * 3 + j];
                }
            }
        }
        float fp[9] = {};
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                for (int k = 0; k < 3; ++k)
                {
                    fp[i * 3 + j] += p_minus[i * 3 + k] * f[j * 3 + k];
                }
            }
        }
        for (int i = 0; i < 9; ++i)
        {
            p_minus[i] = fp[i] + q_[i];
        }

        const float h[6] = {0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
        float s[4] = {};
        for (int i = 0; i < 2; ++i)
        {
            for (int j = 0; j < 2; ++j)
            {
                for (int k = 0; k < 3; ++k)
                {
                    const float hp = h[i * 3 + k];
                    for (int m = 0; m < 3; ++m)
                    {
                        s[i * 2 + j] += hp * p_minus[k * 3 + m] * h[j * 3 + m];
                    }
                }
                s[i * 2 + j] += r_[i * 2 + j];
            }
        }

        const float det = s[0] * s[3] - s[1] * s[2];
        if (std::fabs(det) < 1e-9f)
        {
            x = x_hat_[0];
            v = x_hat_[1];
            return;
        }
        const float inv_s[4] = {s[3] / det, -s[1] / det, -s[2] / det, s[0] / det};

        float ph_t[6] = {};
        for (int i = 0; i < 2; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                for (int k = 0; k < 3; ++k)
                {
                    ph_t[i * 3 + j] += p_minus[j * 3 + k] * h[i * 3 + k];
                }
            }
        }

        float k_gain[6] = {};
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 2; ++j)
            {
                for (int k = 0; k < 2; ++k)
                {
                    k_gain[i * 2 + j] += ph_t[k * 3 + i] * inv_s[k * 2 + j];
                }
            }
        }

        float y[2] = {z[0] - x_minus[1], z[1] - x_minus[2]};
        y[0] = clamp(y[0], -k_max_velocity_innovation, k_max_velocity_innovation);
        y[1] = clamp(y[1], -k_max_accel_innovation, k_max_accel_innovation);
        for (int i = 0; i < 3; ++i)
        {
            x_hat_[i] = x_minus[i];
            for (int j = 0; j < 2; ++j)
            {
                x_hat_[i] += k_gain[i * 2 + j] * y[j];
            }
        }

        x = x_hat_[0];
        v = x_hat_[1];
    }

void command_fusion::reset(const chassis_config& cfg)
{
        msg_ = {};
        msg_.len = cfg.lmin;
        move_enabled_ = false;
        space_prev_ = false;
        space_armed_ = false;
        yaw_slope_.set_default(0.0f);
        yaw_slope_.set_path(0.006f);
        len_target_ = cfg.lmin;
        yaw_ref_initialized_ = false;
        yaw_active_prev_ = false;
    }

void command_fusion::update(const input_snapshot_t& input, const msg_pendulum_t& pendulum, const msg_ins_t& ins,
                const chassis_config& cfg, float dt)
{
        const float max_velocity = std::fabs(cfg.max_cmd_velocity);
        if (input.w && !input.s)
        {
            msg_.v = max_velocity;
        }
        else if (input.s && !input.w)
        {
            msg_.v = -max_velocity;
        }
        else
        {
            msg_.v = 0.0f;
        }
        msg_.v = clamp(msg_.v, -max_velocity, max_velocity);

        const bool yaw_active = input.a != input.d;
        if (!yaw_ref_initialized_)
        {
            msg_.yaw = ins.total_yaw;
            yaw_ref_initialized_ = true;
        }

        if (input.a && !input.d)
        {
            yaw_slope_.update_val(yaw_slope_.value() + 0.0006f);
        }
        else if (input.d && !input.a)
        {
            yaw_slope_.update_val(yaw_slope_.value() - 0.0006f);
        }
        else
        {
            yaw_slope_.set_default(0.0f);
            if (yaw_active_prev_)
            {
                msg_.yaw = ins.total_yaw;
            }
        }
        const float max_yaw_rate = std::fabs(cfg.max_cmd_yaw_rate);
        yaw_slope_.set_default(clamp(yaw_slope_.value(), -max_yaw_rate, max_yaw_rate));
        if (input.q)
        {
            len_target_ = cfg.lmin;
        }
        if (input.e)
        {
            len_target_ = cfg.lmid;
        }
        if (input.f)
        {
            len_target_ = cfg.lmax;
        }

        if (!space_armed_)
        {
            if (!input.space)
            {
                space_armed_ = true;
            }
        }
        else if (input.space && !space_prev_)
        {
            move_enabled_ = !move_enabled_;
        }
        space_prev_ = input.space;

        msg_.move = move_enabled_;
        msg_.dyaw = yaw_slope_.value();
        msg_.len = len_target_;

        if (!pendulum.planar_valid)
        {
            msg_.x = pendulum.x;
        }
        else
        {
            msg_.x += msg_.v * dt;
        }

        if (yaw_active)
        {
            msg_.yaw += msg_.dyaw * dt;
        }
        yaw_active_prev_ = yaw_active;
    }

const msg_cmd_t& command_fusion::msg() const
{ return msg_; }

}  // namespace controller
