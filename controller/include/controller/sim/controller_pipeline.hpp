#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <stdexcept>

#include "control/msgs.hpp"
#include "controller/sim/config.hpp"

namespace controller::sim
{

class PipelineError : public std::runtime_error
{
public:
    using std::runtime_error::runtime_error;
};

struct LinkState
{
    float length = 0.0f;
    float length_rate = 0.0f;
    float phi = 0.0f;
    float phi_rate = 0.0f;
    float alpha = 0.0f;
    float alpha_rate = 0.0f;
    float alpha_equilibrium = 0.0f;
    float normal_force = 0.0f;
    float spring_force = 0.0f;
    float estimated_leg_force = 0.0f;
    float estimated_hip_torque = 0.0f;
};

struct PipelineOutput
{
    std::array<float, 6> raw_torque{};
    control::msg_ins_t imu{};
    control::msg_cmd_t command{};
    control::msg_odometry_t odometry{};
    control::msg_pendulum_t fusion_pendulum{};
    control::msg_pendulum_t pendulum{};
    std::array<float, 10> observed_state{};
    LinkState left{};
    LinkState right{};
    control::chassis_state fsm_state = control::chassis_state::relax;
    std::uint64_t tick_index = 0;
};

class ControllerPipeline
{
public:
    explicit ControllerPipeline(PipelineConfig config = {});
    ~ControllerPipeline();

    ControllerPipeline(const ControllerPipeline&) = delete;
    ControllerPipeline& operator=(const ControllerPipeline&) = delete;
    ControllerPipeline(ControllerPipeline&&) noexcept;
    ControllerPipeline& operator=(ControllerPipeline&&) noexcept;

    PipelineOutput Step(const control::msg_raw_state_t& observation, const control::input_snapshot_t& input);
    void Reset();
    std::uint64_t tick_count() const;
    const PipelineConfig& config() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace controller::sim
