#include "controller/leg.hpp"

#include <cmath>

namespace controller
{

pid::pid(const pid_params& params)
    : pid(params.kp, params.ki, params.kd, params.max_out, params.max_i_out, params.mode)
{
    }

pid::pid(float kp, float ki, float kd, float max_out, float max_i_out, pid_mode mode)
    : kp_(kp), ki_(ki), kd_(kd), max_out_(max_out), max_i_out_(max_i_out), mode_(mode)
{
    }

void pid::tuning(float kp, float ki, float kd)
{
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

void pid::clear()
{
        i_term_ = 0.0f;
        last_fdb_ = 0.0f;
        result_ = 0.0f;
    }

void pid::update_result(float vel)
{
        const float err = ref - fdb;
        p_result_ = kp_ * err;
        if (mode_ == pid_mode::dvel)
        {
            d_result_ = kd_ * vel;
        }
        else
        {
            d_result_ = kd_ * (fdb - last_fdb_);
        }
        i_term_ += ki_ * err;
        i_term_ = clamp(i_term_, -max_i_out_, max_i_out_);
        result_ = clamp(p_result_ + i_term_ + d_result_, -max_out_, max_out_);
        last_fdb_ = fdb;
    }

slope::slope(float val, float path)
    : val_(val), inc_path_(path), dec_path_(path)
{}

void slope::set_default(float val)
{ val_ = val; }

void slope::set_path(float path)
{
        inc_path_ = path;
        dec_path_ = path;
    }

void slope::set_asymmetric(float inc, float dec)
{
        inc_path_ = inc;
        dec_path_ = dec;
    }

float slope::update_val(float target)
{
        const float delta = target - val_;
        if ((delta >= 0.0f && delta < inc_path_) || (delta < 0.0f && delta > -dec_path_))
        {
            val_ = target;
        }
        else if (target < val_)
        {
            val_ -= dec_path_;
        }
        else
        {
            val_ += inc_path_;
        }
        return val_;
    }

float slope::value() const
{ return val_; }

link_solver::link_solver(bool reverse, int j1_motor_idx, int j4_motor_idx, const chassis_config& cfg)
    : reverse_(reverse), j1_motor_idx_(j1_motor_idx), j4_motor_idx_(j4_motor_idx), cfg_(cfg)
{
    }

void link_solver::solve(float pitch, float dpitch, float az, const motor_feedback& j1, const motor_feedback& j4)
{
        const float j1_q = j1.q - cfg_.motor_zero_rad[j1_motor_idx_];
        const float j4_q = j4.q - cfg_.motor_zero_rad[j4_motor_idx_];

        const float phi1 = reverse_ ? (k_pi - j1_q) : (k_pi + j1_q);
        const float phi4 = reverse_ ? (-j4_q) : j4_q;
        const float dphi1 = reverse_ ? (-j1.dq) : j1.dq;
        const float dphi4 = reverse_ ? (-j4.dq) : j4.dq;
        const float joint1_tor = reverse_ ? (-j1.tau) : j1.tau;
        const float joint4_tor = reverse_ ? (-j4.tau) : j4.tau;

        resolve(phi1, phi4);

        const float qdot[2] = {dphi1, dphi4};
        float xdot[2] = {};
        vmc_vel_cal(qdot, xdot);

        len_ = len_kin_;
        dlen_ = xdot[0];
        phi_ = phi_kin_;
        dphi_ = xdot[1];

        if (!phi_init_)
        {
            total_phi_ = phi_;
            last_phi_ = phi_;
            phi_init_ = true;
        }
        else
        {
            total_phi_ += loop_clamp(phi_ - last_phi_, -k_pi, k_pi);
            last_phi_ = phi_;
        }

        alpha_ = loop_clamp(phi_ - 0.5f * k_pi + pitch, -k_pi, k_pi);
        dalpha_ = dphi_ + dpitch;
        alpha_eq_ = cfg_.alpha_eq_coeff[0] + cfg_.alpha_eq_coeff[1] * len_ +
                    cfg_.alpha_eq_coeff[2] * len_ * len_;

        const float treal[2] = {joint1_tor, joint4_tor};
        float trev[2] = {};
        vmc_rev_cal(trev, treal);
        freal_ = trev[0];
        treal_hip_ = trev[1];

        const float cos_alpha = std::cos(alpha_);
        const float sin_alpha = std::sin(alpha_);
        const float p = (trev[0] + fs_) * cos_alpha;
        const float ddlen = dlen_ - prev_dlen_;
        const float ddalpha = dalpha_ - prev_dalpha_;
        const float zw = (az - k_gravity) - ddlen * cos_alpha + 2.0f * dlen_ * dalpha_ * sin_alpha +
                         len_ * ddalpha * sin_alpha + len_ * dalpha_ * dalpha_ * cos_alpha;
        n_ = p + cfg_.mwheel * k_gravity + cfg_.mwheel * zw;

        neutral_ = std::fabs(alpha_) < 0.6f;
        flat_ = (phi_ >= 2.0f && phi_ <= 3.1f);
        prev_dlen_ = dlen_;
        prev_dalpha_ = dalpha_;
    }

void link_solver::vmc_cal(const float f[2], float t[2]) const
{
        t[0] = jt_mat_[0] * f[0] + jt_mat_[1] * f[1];
        t[1] = jt_mat_[2] * f[0] + jt_mat_[3] * f[1];
    }

void link_solver::resolve(float phi1, float phi4)
{
        phi1_ = phi1;
        phi4_ = phi4;

        const float sin1 = std::sin(phi1_);
        const float cos1 = std::cos(phi1_);
        const float sin4 = std::sin(phi4_);
        const float cos4 = std::cos(phi4_);

        const float xdb = cfg_.l1 * (cos4 - cos1);
        const float ydb = cfg_.l1 * (sin4 - sin1);
        const float a0 = 2.0f * cfg_.l2 * xdb;
        const float b0 = 2.0f * cfg_.l2 * ydb;
        const float c0 = xdb * xdb + ydb * ydb;

        u2_ = 2.0f * std::atan2(b0 + std::sqrt(a0 * a0 + b0 * b0 - c0 * c0), a0 + c0);

        coor_b_[0] = cfg_.l1 * cos1;
        coor_b_[1] = cfg_.l1 * sin1;
        coor_c_[0] = coor_b_[0] + cfg_.l2 * std::cos(u2_);
        coor_c_[1] = coor_b_[1] + cfg_.l2 * std::sin(u2_);
        coor_d_[0] = cfg_.l1 * cos4;
        coor_d_[1] = cfg_.l1 * sin4;
        u3_ = k_pi + std::atan2(coor_d_[1] - coor_c_[1], coor_d_[0] - coor_c_[0]);

        phi_kin_ = std::atan2(coor_c_[1], coor_c_[0]);
        len_kin_ = std::sqrt(coor_c_[0] * coor_c_[0] + coor_c_[1] * coor_c_[1]);

        const float sin32 = std::sin(u3_ - u2_);
        const float sin12 = std::sin(phi1_ - u2_);
        const float sin34 = std::sin(u3_ - phi4_);
        const float cos03 = std::cos(phi_kin_ - u3_);
        const float cos02 = std::cos(phi_kin_ - u2_);
        const float sin03 = std::sin(phi_kin_ - u3_);
        const float sin02 = std::sin(phi_kin_ - u2_);

        j_mat_[0] = cfg_.l1 * sin03 * sin12 / sin32;
        j_mat_[1] = cfg_.l1 * sin02 * sin34 / sin32;
        j_mat_[2] = cfg_.l1 * cos03 * sin12 / (sin32 * len_kin_);
        j_mat_[3] = cfg_.l1 * cos02 * sin34 / (sin32 * len_kin_);

        jt_mat_[0] = j_mat_[0];
        jt_mat_[1] = j_mat_[2];
        jt_mat_[2] = j_mat_[1];
        jt_mat_[3] = j_mat_[3];

        jt_inv_mat_[0] = -cos02 / (sin12 * cfg_.l1);
        jt_inv_mat_[1] = cos03 / (sin34 * cfg_.l1);
        jt_inv_mat_[2] = sin02 * len_kin_ / (sin12 * cfg_.l1);
        jt_inv_mat_[3] = -sin03 * len_kin_ / (sin34 * cfg_.l1);

        calc_spring_force();
    }

void link_solver::calc_spring_force()
{
        const float ang_p = phi1_ + cfg_.ang_spring - k_pi * 0.5f;
        const float px = cfg_.dspring1 * std::cos(ang_p);
        const float py = cfg_.dspring1 * std::sin(ang_p);

        const float cos1 = std::cos(phi1_);
        const float sin1 = std::sin(phi1_);
        const float cos2 = std::cos(u2_);
        const float sin2 = std::sin(u2_);
        const float qx = cfg_.l1 * cos1 + cfg_.dspring2 * cos2;
        const float qy = cfg_.l1 * sin1 + cfg_.dspring2 * sin2;

        const float dpqx = qx - px;
        const float dpqy = qy - py;
        const float ls = std::sqrt(dpqx * dpqx + dpqy * dpqy);
        if (ls < 1e-5f)
        {
            fs_ = 0.0f;
            return;
        }

        const float fsx = cfg_.fspring * dpqx / ls;
        const float fsy = cfg_.fspring * dpqy / ls;

        const float s23 = std::sin(u3_ - u2_);
        if (std::fabs(s23) < 0.05f)
        {
            fs_ = 0.0f;
            return;
        }

        const float inv_l2s23 = 1.0f / (cfg_.l2 * s23);
        const float dphi2_dphi1 = cfg_.l1 * std::sin(u3_ - phi1_) * inv_l2s23;
        const float dphi2_dphi4 = cfg_.l1 * std::sin(phi4_ - u3_) * inv_l2s23;

        const float dpx_d1 = -cfg_.dspring1 * std::sin(ang_p);
        const float dpy_d1 = cfg_.dspring1 * std::cos(ang_p);
        const float dqx_d1 = -cfg_.l1 * sin1 - cfg_.dspring2 * sin2 * dphi2_dphi1;
        const float dqy_d1 = cfg_.l1 * cos1 + cfg_.dspring2 * cos2 * dphi2_dphi1;
        const float dqx_d4 = -cfg_.dspring2 * sin2 * dphi2_dphi4;
        const float dqy_d4 = cfg_.dspring2 * cos2 * dphi2_dphi4;

        const float ddx_d1 = dqx_d1 - dpx_d1;
        const float ddy_d1 = dqy_d1 - dpy_d1;
        const float tau_s1 = fsx * ddx_d1 + fsy * ddy_d1;
        const float tau_s4 = fsx * dqx_d4 + fsy * dqy_d4;

        fs_ = jt_inv_mat_[0] * tau_s1 + jt_inv_mat_[1] * tau_s4;
    }

void link_solver::vmc_rev_cal(float f[2], const float t[2]) const
{
        f[0] = jt_inv_mat_[0] * t[0] + jt_inv_mat_[1] * t[1];
        f[1] = jt_inv_mat_[2] * t[0] + jt_inv_mat_[3] * t[1];
    }

void link_solver::vmc_vel_cal(const float phi_dot[2], float v_dot[2]) const
{
        v_dot[0] = j_mat_[0] * phi_dot[0] + j_mat_[1] * phi_dot[1];
        v_dot[1] = j_mat_[2] * phi_dot[0] + j_mat_[3] * phi_dot[1];
    }

leg_controller::leg_controller(bool reverse, int j1_motor_idx, int j4_motor_idx, const chassis_config& cfg)
    : reverse_(reverse),
          cfg_(cfg),
          link_(reverse, j1_motor_idx, j4_motor_idx, cfg),
          len_pd_(cfg.leg_pid.len),
          phi_pd_(cfg.leg_pid.phi)
{
    }

bool leg_controller::reverse() const
{ return reverse_; }

float leg_controller::hip_sign() const
{ return reverse_ ? -1.0f : 1.0f; }

float leg_controller::wheel_sign() const
{ return reverse_ ? -1.0f : 1.0f; }

link_solver& leg_controller::link()
{ return link_; }

const link_solver& leg_controller::link() const
{ return link_; }

void leg_controller::relax()
{
        delta_init_ = false;
        len_pd_.clear();
        phi_pd_.clear();
    }

float leg_controller::phi_control(float target_phi, float kp, float kd, float slope_path, bool positive)
{
        if (!delta_init_)
        {
            phi_updater_.set_default(link_.total_phi_);
            phi_updater_.set_path(slope_path);
            target_phi_ = target_phi;
            while (target_phi_ - link_.total_phi_ > k_pi)
            {
                target_phi_ -= k_pi * 2.0f;
            }
            while (target_phi_ - link_.total_phi_ < -k_pi)
            {
                target_phi_ += k_pi * 2.0f;
            }
            if (positive && target_phi_ < link_.total_phi_)
            {
                target_phi_ += k_pi * 2.0f;
            }
            else if (!positive && target_phi_ > link_.total_phi_)
            {
                target_phi_ -= k_pi * 2.0f;
            }
            delta_init_ = true;
        }

        phi_pd_.tuning(kp, 0.0f, kd);
        phi_pd_.ref = phi_updater_.update_val(target_phi_);
        phi_pd_.fdb = link_.total_phi_;
        phi_pd_.update_result(link_.dphi_);
        return phi_pd_.result_;
    }

float leg_controller::len_control(float ref)
{
        len_pd_.ref = ref;
        len_pd_.fdb = link_.len_;
        len_pd_.update_result(link_.dlen_);
        return len_pd_.result_;
    }

void leg_controller::apply_torque(const float f[2], float tw, float& j1_tau, float& j4_tau, float& wheel_tau) const
{
        float t[2] = {};
        link_.vmc_cal(f, t);
        j1_tau = clamp(t[0], -cfg_.thip_max, cfg_.thip_max) * hip_sign();
        j4_tau = clamp(t[1], -cfg_.thip_max, cfg_.thip_max) * hip_sign();
        wheel_tau = clamp(tw, -cfg_.twheel_max, cfg_.twheel_max) * wheel_sign();
    }

void leg_controller::tune_len_pd(float kp, float ki, float kd)
{ len_pd_.tuning(kp, ki, kd); }

}  // namespace controller
