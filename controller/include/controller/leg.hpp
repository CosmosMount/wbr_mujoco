#pragma once

#include "controller/config.hpp"
#include "controller/math.hpp"
#include "controller/msgs.hpp"

#include <cmath>

namespace controller
{

class pid
{
public:
    explicit pid(const pid_params& params);

    pid(float kp, float ki, float kd, float max_out, float max_i_out, pid_mode mode = pid_mode::position);

    void tuning(float kp, float ki, float kd);

    void clear();

    void update_result(float vel = 0.0f);

    float ref = 0.0f;
    float fdb = 0.0f;
    float result_ = 0.0f;

private:
    float kp_ = 0.0f;
    float ki_ = 0.0f;
    float kd_ = 0.0f;
    float max_out_ = 0.0f;
    float max_i_out_ = 0.0f;
    pid_mode mode_ = pid_mode::position;
    float p_result_ = 0.0f;
    float i_term_ = 0.0f;
    float d_result_ = 0.0f;
    float last_fdb_ = 0.0f;
};

class slope
{
public:
    slope(float val, float path);

    void set_default(float val);
    void set_path(float path);

    void set_asymmetric(float inc, float dec);

    float update_val(float target);

    float value() const;

private:
    float val_ = 0.0f;
    float inc_path_ = 0.0f;
    float dec_path_ = 0.0f;
};

class link_solver
{
public:
    link_solver(bool reverse, int j1_motor_idx, int j4_motor_idx, const chassis_config& cfg);

    // Sign convention (left view, CCW positive):
    //   phi1/phi4, phi, alpha — CCW positive
    //   F[0] — virtual leg-axis force; positive extends leg length
    //   F[1] — hip torque in VMC; positive is CCW
    //   Motor tau — MuJoCo actuator sign; hip/wheel scaled by reverse only in apply_torque()
    void solve(float pitch, float dpitch, float az, const motor_feedback& j1, const motor_feedback& j4);

    void vmc_cal(const float f[2], float t[2]) const;

    float phi_ = 0.0f;
    float dphi_ = 0.0f;
    float alpha_ = 0.0f;
    float dalpha_ = 0.0f;
    float alpha_eq_ = 0.0f;
    float len_ = 0.0f;
    float dlen_ = 0.0f;
    float n_ = 0.0f;
    float freal_ = 0.0f;
    float treal_hip_ = 0.0f;
    float fs_ = 0.0f;
    float total_phi_ = 0.0f;
    bool flat_ = false;
    bool neutral_ = false;

private:
    void resolve(float phi1, float phi4);

    void calc_spring_force();

    void vmc_rev_cal(float f[2], const float t[2]) const;

    void vmc_vel_cal(const float phi_dot[2], float v_dot[2]) const;

    bool reverse_ = false;
    int j1_motor_idx_ = 0;
    int j4_motor_idx_ = 1;
    chassis_config cfg_;
    float phi1_ = 0.0f;
    float phi4_ = 0.0f;
    float len_kin_ = 0.0f;
    float phi_kin_ = k_pi / 2.0f;
    float u2_ = 0.0f;
    float u3_ = 0.0f;
    float coor_b_[2] = {};
    float coor_c_[2] = {};
    float coor_d_[2] = {};
    float j_mat_[4] = {};
    float jt_mat_[4] = {};
    float jt_inv_mat_[4] = {};
    float prev_dlen_ = 0.0f;
    float prev_dalpha_ = 0.0f;
    float last_phi_ = 0.0f;
    bool phi_init_ = false;
};

class leg_controller
{
public:
    leg_controller(bool reverse, int j1_motor_idx, int j4_motor_idx, const chassis_config& cfg);

    bool reverse() const;

    float hip_sign() const;

    float wheel_sign() const;

    link_solver& link();
    const link_solver& link() const;

    void relax();

    float phi_control(float target_phi, float kp, float kd, float slope_path, bool positive);

    float len_control(float ref);

    void apply_torque(const float f[2], float tw, float& j1_tau, float& j4_tau, float& wheel_tau) const;

    void tune_len_pd(float kp, float ki, float kd);

    bool delta_init_ = false;

private:
    bool reverse_ = false;
    chassis_config cfg_;
    link_solver link_;
    pid len_pd_;
    pid phi_pd_;
    slope phi_updater_{0.0f, 0.005f};
    float target_phi_ = 0.0f;
};

}  // namespace controller
