#include "controller/sim/tick_controller.hpp"

#include <utility>

namespace controller::sim {

mujoco_sim::adapter_sdk::AdapterOptions MakeAdapterOptions(
    const AdapterConfig& config) {
  mujoco_sim::adapter_sdk::AdapterOptions options;
  options.server_name = config.server_name;
  options.controller_id = config.controller_id;
  options.observation_names = ObservationNames(config.interfaces);
  options.command_names = CommandNames(config.interfaces);
  options.timeout_ms = config.timeout_ms;
  options.service_timeout_ms = config.service_timeout_ms;
  return options;
}

TickController::TickController(
    const mujoco_sim::v1::ResourceLayout& registered_layout,
    PipelineConfig pipeline_config, InputScript input_script)
    : TickController(registered_layout, MakeDefaultInterfaceConfig(),
                     std::move(pipeline_config), std::move(input_script)) {}

TickController::TickController(
    const mujoco_sim::v1::ResourceLayout& registered_layout,
    InterfaceConfig interface_config, PipelineConfig pipeline_config,
    InputScript input_script)
    : decoder_(interface_config, registered_layout),
      command_mapper_(std::move(interface_config), registered_layout),
      pipeline_(std::move(pipeline_config)),
      input_script_(std::move(input_script)) {}

std::vector<mujoco_sim::adapter_sdk::CommandValue>
TickController::HandleTick(const mujoco_sim::v1::Tick& tick) {
  return HandleTick(tick, input_script_.InputAt(pipeline_.tick_count()));
}

std::vector<mujoco_sim::adapter_sdk::CommandValue>
TickController::HandleTick(const mujoco_sim::v1::Tick& tick,
                           const control::input_snapshot_t& input) {
  const control::msg_raw_state_t raw = decoder_.Decode(tick);
  last_input_ = input;
  last_output_ = pipeline_.Step(raw, input);
  return command_mapper_.Map(last_output_->raw_torque);
}

void TickController::Reset() {
  pipeline_.Reset();
  last_input_ = {};
  last_output_.reset();
}

}  // namespace controller::sim
