#pragma once

// Portable 6-axis IMU attitude fusion (Mahony complementary filter).
// Dependencies: <cmath> only — suitable for MCU firmware (copy this header).
// Frame: body-fixed IMU, quaternion maps body -> world (w, x, y, z).
// Accel at rest should read specific force opposite gravity (e.g. +Z when upright).

#include <cmath>

namespace controller
{

struct imu_vec3
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct imu_attitude_t
{
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // w, x, y, z
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float total_yaw = 0.0f;
};

struct mahony_config
{
    float kp = 2.0f;
    float ki = 0.0f;
    float accel_norm_min = 1e-3f;
};

class mahony_filter
{
public:
    imu_attitude_t attitude{};

    void reset();

    void configure(const mahony_config& cfg);

    void set_kp(float kp);

    void set_ki(float ki);

    bool init_from_accel(float ax, float ay, float az);

    void update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

private:
    static float clamp(float v, float lo, float hi);

    static void quat_normalize(float q[4]);

    static void gravity_body_from_quat(const float q[4], imu_vec3& out);

    static bool quat_from_accel(float ax, float ay, float az, float q[4]);

    static void quat_to_euler(const float q[4], float& roll, float& pitch, float& yaw);

    void update_euler();

    mahony_config cfg_{};
    imu_vec3 integral_{};
    float prev_yaw_ = 0.0f;
    int yaw_round_count_ = 0;
    bool initialized_ = false;
};

}  // namespace controller
