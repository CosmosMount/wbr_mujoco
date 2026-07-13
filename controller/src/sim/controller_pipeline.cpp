#include "controller/sim/controller_pipeline.hpp"

#include <cmath>
#include <cstring>
#include <iterator>
#include <limits>
#include <sstream>
#include <string>
#include <utility>

#include <control/balance.hpp>
#include <control/chassis_fsm.hpp>
#include <control/imu_fusion.hpp>
#include <control/leg.hpp>
#include <control/math.hpp>

namespace controller::sim {
namespace {

void RequireFinite(float value, const std::string& field) {
  if (!std::isfinite(value)) {
    throw PipelineError("non-finite pipeline value: " + field);
  }
}

void ValidateRawObservation(const control::msg_raw_state_t& raw) {
  if (!std::isfinite(raw.time)) {
    throw PipelineError("non-finite observation value: time");
  }
  for (int i = 0; i < 3; ++i) {
    RequireFinite(raw.gyro[i], "gyro[" + std::to_string(i) + "]");
    RequireFinite(raw.accel[i], "accel[" + std::to_string(i) + "]");
  }
  for (int i = 0; i < 4; ++i) {
    RequireFinite(raw.quat_gt[i], "quat_gt[" + std::to_string(i) + "]");
  }
  for (int i = 0; i < 6; ++i) {
    RequireFinite(raw.motors[i].q,
                  "motors[" + std::to_string(i) + "].q");
    RequireFinite(raw.motors[i].dq,
                  "motors[" + std::to_string(i) + "].dq");
    RequireFinite(raw.motors[i].tau,
                  "motors[" + std::to_string(i) + "].tau");
  }
}

void ValidateImu(const control::msg_ins_t& imu) {
  for (int i = 0; i < 4; ++i) {
    RequireFinite(imu.quaternion[i],
                  "imu.quaternion[" + std::to_string(i) + "]");
  }
  RequireFinite(imu.roll, "imu.roll");
  RequireFinite(imu.pitch, "imu.pitch");
  RequireFinite(imu.yaw, "imu.yaw");
  RequireFinite(imu.total_yaw, "imu.total_yaw");
  RequireFinite(imu.gyro_r, "imu.gyro_r");
  RequireFinite(imu.gyro_p, "imu.gyro_p");
  RequireFinite(imu.gyro_y, "imu.gyro_y");
  for (int i = 0; i < 3; ++i) {
    RequireFinite(imu.accel[i], "imu.accel[" + std::to_string(i) + "]");
  }
}

LinkState SnapshotLink(const control::link_solver& link,
                       const std::string& side) {
  LinkState state{
      link.len_,       link.dlen_,     link.phi_,       link.dphi_,
      link.alpha_,     link.dalpha_,   link.alpha_eq_,  link.n_,
      link.fs_,        link.freal_,    link.treal_hip_,
  };
  const float values[] = {
      state.length,              state.length_rate,
      state.phi,                 state.phi_rate,
      state.alpha,               state.alpha_rate,
      state.alpha_equilibrium,   state.normal_force,
      state.spring_force,        state.estimated_leg_force,
      state.estimated_hip_torque,
  };
  for (std::size_t i = 0; i < std::size(values); ++i) {
    RequireFinite(values[i], side + " link field " + std::to_string(i));
  }
  return state;
}

void ValidateCommand(const control::msg_cmd_t& command) {
  const float values[] = {command.x,     command.v,    command.len,
                          command.dlen,  command.yaw,  command.dyaw,
                          command.roll,  command.tri_spd};
  for (std::size_t i = 0; i < std::size(values); ++i) {
    RequireFinite(values[i], "command field " + std::to_string(i));
  }
}

control::msg_ins_t BuildImu(control::mahony_filter& mahony,
                            control::imu_mode mode,
                            const control::msg_raw_state_t& raw) {
  control::msg_ins_t imu{};
  if (mode == control::imu_mode::mahony) {
    mahony.update(raw.gyro[0], raw.gyro[1], raw.gyro[2], raw.accel[0],
                  raw.accel[1], raw.accel[2], kControlDtSeconds);
    for (int i = 0; i < 4; ++i) {
      imu.quaternion[i] = mahony.attitude.q[i];
    }
    imu.roll = mahony.attitude.roll;
    imu.pitch = mahony.attitude.pitch;
    imu.yaw = mahony.attitude.yaw;
    imu.total_yaw = mahony.attitude.total_yaw;
  } else if (mode == control::imu_mode::bypass) {
    for (int i = 0; i < 4; ++i) {
      imu.quaternion[i] = raw.quat_gt[i];
    }
    control::quat_to_euler(imu.quaternion, imu.roll, imu.pitch, imu.yaw);
    imu.total_yaw = imu.yaw;
  } else {
    throw PipelineError("unsupported IMU mode");
  }

  imu.gyro_r = raw.gyro[0];
  imu.gyro_p = raw.gyro[1];
  imu.gyro_y = raw.gyro[2];
  for (int i = 0; i < 3; ++i) {
    imu.accel[i] = raw.accel[i];
  }
  ValidateImu(imu);
  return imu;
}

}  // namespace

struct ControllerPipeline::Impl {
  explicit Impl(PipelineConfig pipeline_config)
      : config(std::move(pipeline_config)),
        left(true, 0, 1, config.chassis),
        right(false, 3, 4, config.chassis) {
    if (!std::isfinite(config.chassis.control_dt) ||
        std::fabs(config.chassis.control_dt - kControlDtSeconds) >
            std::numeric_limits<float>::epsilon()) {
      throw PipelineError("chassis.control_dt must be exactly 0.001 seconds");
    }
    mahony.reset();
    mahony.configure(config.mahony);
    fusion.reset(config.chassis);
    fsm.init(config.chassis);
  }

  PipelineConfig config;
  control::mahony_filter mahony;
  control::command_fusion fusion;
  control::leg_controller left;
  control::leg_controller right;
  control::odometry odometry;
  control::chassis_fsm fsm;
  control::msg_pendulum_t previous_pendulum{};
  std::uint64_t tick_count = 0;
};

ControllerPipeline::ControllerPipeline(PipelineConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

ControllerPipeline::~ControllerPipeline() = default;
ControllerPipeline::ControllerPipeline(ControllerPipeline&&) noexcept = default;
ControllerPipeline& ControllerPipeline::operator=(ControllerPipeline&&) noexcept =
    default;

PipelineOutput ControllerPipeline::Step(
    const control::msg_raw_state_t& observation,
    const control::input_snapshot_t& input) {
  ValidateRawObservation(observation);

  PipelineOutput output{};
  output.tick_index = impl_->tick_count;
  output.fusion_pendulum = impl_->previous_pendulum;
  output.imu = BuildImu(impl_->mahony, impl_->config.imu_mode, observation);

  impl_->fusion.update(input, impl_->previous_pendulum, output.imu,
                       impl_->config.chassis, kControlDtSeconds);
  output.command = impl_->fusion.msg();
  ValidateCommand(output.command);

  const float pitch = output.imu.pitch;
  const float pitch_rate = output.imu.gyro_p;
  const float yaw = output.imu.total_yaw;
  const float yaw_rate = output.imu.gyro_y;
  const float left_velocity = observation.motors[2].dq *
                              impl_->config.chassis.rwheel *
                              impl_->left.wheel_sign();
  const float right_velocity = observation.motors[5].dq *
                               impl_->config.chassis.rwheel *
                               impl_->right.wheel_sign();

  impl_->left.link().solve(pitch, pitch_rate, 0.0F,
                           observation.motors[0], observation.motors[1]);
  impl_->right.link().solve(pitch, pitch_rate, 0.0F,
                            observation.motors[3], observation.motors[4]);
  SnapshotLink(impl_->left.link(), "left first solve");
  SnapshotLink(impl_->right.link(), "right first solve");

  impl_->odometry.update(output.imu.quaternion, output.imu.accel,
                         (left_velocity + right_velocity) * 0.5F, yaw,
                         kControlDtSeconds);
  output.odometry = {impl_->odometry.x, impl_->odometry.v,
                     impl_->odometry.az};
  RequireFinite(output.odometry.x, "odometry.x");
  RequireFinite(output.odometry.v, "odometry.v");
  RequireFinite(output.odometry.a_z, "odometry.a_z");

  impl_->left.link().solve(pitch, pitch_rate, impl_->odometry.az,
                           observation.motors[0], observation.motors[1]);
  impl_->right.link().solve(pitch, pitch_rate, impl_->odometry.az,
                            observation.motors[3], observation.motors[4]);
  output.left = SnapshotLink(impl_->left.link(), "left second solve");
  output.right = SnapshotLink(impl_->right.link(), "right second solve");

  output.observed_state = {
      impl_->odometry.x, impl_->odometry.v, yaw, yaw_rate,
      output.left.alpha, output.left.alpha_rate,
      output.right.alpha, output.right.alpha_rate, pitch, pitch_rate,
  };

  control::fsm_inputs fsm_input{output.imu, output.command, output.odometry,
                                impl_->left, impl_->right};
  std::memcpy(fsm_input.observed_x, output.observed_state.data(),
              sizeof(fsm_input.observed_x));
  fsm_input.n_total = output.left.normal_force + output.right.normal_force;
  fsm_input.chassis_dead = false;

  control::fsm_outputs fsm_output{};
  impl_->fsm.step(fsm_input, fsm_output);
  if (!output.command.move || impl_->config.chassis.force_relax) {
    fsm_output.motor = {};
  }

  for (std::size_t i = 0; i < output.raw_torque.size(); ++i) {
    RequireFinite(fsm_output.motor.tau[i],
                  "raw_torque[" + std::to_string(i) + "]");
    output.raw_torque[i] = fsm_output.motor.tau[i];
  }
  output.pendulum = fsm_output.pendulum;
  output.fsm_state = impl_->fsm.state();

  impl_->previous_pendulum = output.pendulum;
  ++impl_->tick_count;
  return output;
}

void ControllerPipeline::Reset() {
  PipelineConfig config = impl_->config;
  impl_ = std::make_unique<Impl>(std::move(config));
}

std::uint64_t ControllerPipeline::tick_count() const {
  return impl_->tick_count;
}

const PipelineConfig& ControllerPipeline::config() const {
  return impl_->config;
}

}  // namespace controller::sim
