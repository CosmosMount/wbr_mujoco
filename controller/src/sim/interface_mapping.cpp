#include "controller/sim/interface_mapping.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace controller::sim {
namespace {

struct ResourceSpec {
  std::string name;
  std::uint32_t dimension = 1;
};

std::vector<ResourceSpec> ObservationSpecs(const InterfaceConfig& config) {
  std::vector<ResourceSpec> specs = {
      {config.imu.quaternion, 4},
      {config.imu.linear_acceleration, 3},
      {config.imu.angular_velocity, 3},
  };
  specs.reserve(kObservationResourceCount);
  for (const auto& motor : config.motors) specs.push_back({motor.position_state, 1});
  for (const auto& motor : config.motors) specs.push_back({motor.velocity_state, 1});
  for (const auto& motor : config.motors) specs.push_back({motor.effort_state, 1});
  return specs;
}

std::array<std::string, 6> ConfiguredCommandNames(const InterfaceConfig& config) {
  std::array<std::string, 6> names;
  for (std::size_t i = 0; i < names.size(); ++i) {
    names[i] = config.motors[i].effort_command;
  }
  return names;
}

void ValidateInterfaceConfig(const InterfaceConfig& config) {
  const auto observations = ObservationSpecs(config);
  const auto commands = ConfiguredCommandNames(config);
  std::unordered_set<std::string> observation_names;
  for (const auto& spec : observations) {
    if (spec.name.empty()) throw InterfaceError("observation resource name is empty");
    if (!observation_names.insert(spec.name).second) {
      throw InterfaceError("duplicate configured observation resource: " + spec.name);
    }
  }
  std::unordered_set<std::string> command_names;
  std::unordered_set<std::string> semantic_names;
  for (std::size_t i = 0; i < config.motors.size(); ++i) {
    if (config.motors[i].semantic_name.empty()) {
      throw InterfaceError("motor semantic name is empty at index " + std::to_string(i));
    }
    if (!semantic_names.insert(config.motors[i].semantic_name).second) {
      throw InterfaceError("duplicate motor semantic name: " +
                           config.motors[i].semantic_name);
    }
    if (commands[i].empty()) throw InterfaceError("command resource name is empty");
    if (!command_names.insert(commands[i]).second) {
      throw InterfaceError("duplicate configured command resource: " + commands[i]);
    }
  }
}

const ResourceSpec* FindObservationSpec(const std::vector<ResourceSpec>& specs,
                                        std::string_view name) {
  const auto found = std::find_if(specs.begin(), specs.end(),
                                  [name](const ResourceSpec& spec) {
                                    return spec.name == name;
                                  });
  return found == specs.end() ? nullptr : &*found;
}

std::size_t CommandIndex(const std::array<std::string, 6>& names,
                         std::string_view name) {
  const auto found = std::find(names.begin(), names.end(), name);
  if (found == names.end()) {
    throw InterfaceError("unexpected command resource: " + std::string(name));
  }
  return static_cast<std::size_t>(found - names.begin());
}

const mujoco_sim::v1::ResourceDescriptor& FindCommandDescriptor(
    const mujoco_sim::v1::ResourceLayout& layout, std::string_view name) {
  const auto found = std::find_if(
      layout.commands().begin(), layout.commands().end(),
      [name](const mujoco_sim::v1::ResourceDescriptor& descriptor) {
        return descriptor.name() == name;
      });
  if (found == layout.commands().end()) {
    throw InterfaceError("missing command resource: " + std::string(name));
  }
  return *found;
}

void ValidateTickLayout(const InterfaceConfig& config,
                        const mujoco_sim::v1::ResourceLayout& tick_layout,
                        const mujoco_sim::v1::ResourceLayout& registered_layout) {
  ValidateLayout(config, tick_layout);
  for (const auto& name : ConfiguredCommandNames(config)) {
    const auto& tick_descriptor = FindCommandDescriptor(tick_layout, name);
    const auto& registered_descriptor = FindCommandDescriptor(registered_layout, name);
    if (tick_descriptor.has_ctrl_range() != registered_descriptor.has_ctrl_range() ||
        tick_descriptor.ctrl_min() != registered_descriptor.ctrl_min() ||
        tick_descriptor.ctrl_max() != registered_descriptor.ctrl_max()) {
      throw InterfaceError("tick ctrlrange differs from registration for " + name);
    }
  }
}

float ToFiniteFloat(double value, const std::string& resource, int component) {
  if (!std::isfinite(value) ||
      value > static_cast<double>(std::numeric_limits<float>::max()) ||
      value < -static_cast<double>(std::numeric_limits<float>::max())) {
    throw InterfaceError("non-finite or out-of-range observation value in " +
                         resource + "[" + std::to_string(component) + "]");
  }
  return static_cast<float>(value);
}

using BlockMap =
    std::unordered_map<std::string, const mujoco_sim::v1::ObservationBlock*>;

const mujoco_sim::v1::ObservationBlock& GetBlock(const BlockMap& blocks,
                                                  const std::string& name) {
  const auto found = blocks.find(name);
  if (found == blocks.end()) {
    throw InterfaceError("missing observation block: " + name);
  }
  return *found->second;
}

float GetScalar(const BlockMap& blocks, const std::string& name) {
  return ToFiniteFloat(GetBlock(blocks, name).values(0), name, 0);
}

}  // namespace

std::vector<std::string> ObservationNames() {
  return ObservationNames(MakeDefaultInterfaceConfig());
}

std::vector<std::string> ObservationNames(const InterfaceConfig& config) {
  ValidateInterfaceConfig(config);
  std::vector<std::string> names;
  for (const auto& spec : ObservationSpecs(config)) names.push_back(spec.name);
  return names;
}

std::vector<std::string> CommandNames() {
  return CommandNames(MakeDefaultInterfaceConfig());
}

std::vector<std::string> CommandNames(const InterfaceConfig& config) {
  ValidateInterfaceConfig(config);
  const auto configured = ConfiguredCommandNames(config);
  return {configured.begin(), configured.end()};
}

void ValidateLayout(const mujoco_sim::v1::ResourceLayout& layout) {
  ValidateLayout(MakeDefaultInterfaceConfig(), layout);
}

void ValidateLayout(const InterfaceConfig& config,
                       const mujoco_sim::v1::ResourceLayout& layout) {
  ValidateInterfaceConfig(config);
  const auto specs = ObservationSpecs(config);
  const auto commands = ConfiguredCommandNames(config);
  if (layout.observations_size() != static_cast<int>(specs.size())) {
    throw InterfaceError("observation layout must contain exactly 21 resources");
  }
  std::unordered_set<std::string> names;
  for (const auto& descriptor : layout.observations()) {
    const ResourceSpec* spec = FindObservationSpec(specs, descriptor.name());
    if (spec == nullptr) {
      throw InterfaceError("unexpected observation resource: " + descriptor.name());
    }
    if (!names.insert(descriptor.name()).second) {
      throw InterfaceError("duplicate observation resource: " + descriptor.name());
    }
    if (descriptor.type() != mujoco_sim::v1::OBSERVATION_SENSOR ||
        descriptor.dimension() != spec->dimension) {
      throw InterfaceError("observation layout mismatch for " + descriptor.name());
    }
  }
  if (layout.commands_size() != static_cast<int>(commands.size())) {
    throw InterfaceError("command layout must contain exactly 6 resources");
  }
  names.clear();
  for (const auto& descriptor : layout.commands()) {
    CommandIndex(commands, descriptor.name());
    if (!names.insert(descriptor.name()).second) {
      throw InterfaceError("duplicate command resource: " + descriptor.name());
    }
    if (descriptor.type() != mujoco_sim::v1::COMMAND_ACTUATOR ||
        descriptor.dimension() != 1 || !descriptor.has_ctrl_range() ||
        !std::isfinite(descriptor.ctrl_min()) ||
        !std::isfinite(descriptor.ctrl_max()) ||
        descriptor.ctrl_min() > descriptor.ctrl_max()) {
      throw InterfaceError("invalid command layout for " + descriptor.name());
    }
  }
}

TickDecoder::TickDecoder(const mujoco_sim::v1::ResourceLayout& registered_layout)
    : TickDecoder(MakeDefaultInterfaceConfig(), registered_layout) {}

TickDecoder::TickDecoder(
    InterfaceConfig config,
    const mujoco_sim::v1::ResourceLayout& registered_layout)
    : config_(std::move(config)), registered_layout_(registered_layout) {
  ValidateLayout(config_, registered_layout_);
}

control::msg_raw_state_t TickDecoder::Decode(
    const mujoco_sim::v1::Tick& tick) const {
  ValidateTickLayout(config_, tick.layout(), registered_layout_);
  const auto specs = ObservationSpecs(config_);
  if (tick.observations_size() != static_cast<int>(specs.size())) {
    throw InterfaceError("tick must contain exactly 21 observation blocks");
  }
  BlockMap blocks;
  for (const auto& block : tick.observations()) {
    const ResourceSpec* spec = FindObservationSpec(specs, block.name());
    if (spec == nullptr) throw InterfaceError("unexpected observation block: " + block.name());
    if (!blocks.emplace(block.name(), &block).second) {
      throw InterfaceError("duplicate observation block: " + block.name());
    }
    if (block.values_size() != static_cast<int>(spec->dimension)) {
      throw InterfaceError("observation value dimension mismatch for " + block.name());
    }
  }

  control::msg_raw_state_t raw{};
  raw.time = static_cast<double>(tick.header().tick_id()) *
             static_cast<double>(kControlDtSeconds);
  const auto& quaternion = GetBlock(blocks, config_.imu.quaternion);
  const auto& acceleration = GetBlock(blocks, config_.imu.linear_acceleration);
  const auto& angular_velocity = GetBlock(blocks, config_.imu.angular_velocity);
  for (int i = 0; i < 4; ++i) {
    raw.quat_gt[i] = ToFiniteFloat(quaternion.values(i), config_.imu.quaternion, i);
  }
  for (int i = 0; i < 3; ++i) {
    raw.accel[i] = ToFiniteFloat(acceleration.values(i),
                                 config_.imu.linear_acceleration, i);
    raw.gyro[i] = ToFiniteFloat(angular_velocity.values(i),
                                config_.imu.angular_velocity, i);
  }
  for (std::size_t i = 0; i < config_.motors.size(); ++i) {
    raw.motors[i].q = GetScalar(blocks, config_.motors[i].position_state);
    raw.motors[i].dq = GetScalar(blocks, config_.motors[i].velocity_state);
    raw.motors[i].tau = GetScalar(blocks, config_.motors[i].effort_state);
  }
  return raw;
}

TorqueCommandMapper::TorqueCommandMapper(
    const mujoco_sim::v1::ResourceLayout& registered_layout)
    : TorqueCommandMapper(MakeDefaultInterfaceConfig(), registered_layout) {}

TorqueCommandMapper::TorqueCommandMapper(
    InterfaceConfig config,
    const mujoco_sim::v1::ResourceLayout& registered_layout) {
  ValidateLayout(config, registered_layout);
  command_names_ = ConfiguredCommandNames(config);
  std::size_t order = 0;
  for (const auto& descriptor : registered_layout.commands()) {
    const std::size_t motor_index = CommandIndex(command_names_, descriptor.name());
    ranges_[motor_index] = {descriptor.ctrl_min(), descriptor.ctrl_max()};
    registered_order_[order++] = motor_index;
  }
}

std::vector<mujoco_sim::adapter_sdk::CommandValue> TorqueCommandMapper::Map(
    const std::array<float, 6>& raw_torque) const {
  for (std::size_t i = 0; i < raw_torque.size(); ++i) {
    if (!std::isfinite(raw_torque[i])) {
      throw InterfaceError("non-finite raw torque at motor index " +
                           std::to_string(i));
    }
  }
  std::vector<mujoco_sim::adapter_sdk::CommandValue> commands;
  last_clamp_count_ = 0;
  for (const std::size_t motor_index : registered_order_) {
    const double raw_value = static_cast<double>(raw_torque[motor_index]);
    const double clamped =
        std::clamp(raw_value, ranges_[motor_index].minimum,
                   ranges_[motor_index].maximum);
    if (clamped != raw_value) ++last_clamp_count_;
    commands.push_back(
        {command_names_[motor_index], clamped});
  }
  return commands;
}

}  // namespace controller::sim
