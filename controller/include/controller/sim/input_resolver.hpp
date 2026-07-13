#pragma once

#include <chrono>
#include <optional>

#include "control/msgs.hpp"
#include "controller/sim/input_script.hpp"

#include <mujoco_sim/adapter_sdk/operator_input_client.hpp>
#include <mujoco_sim/v1/sync.pb.h>

namespace controller::sim
{

control::input_snapshot_t ToInputSnapshot(const mujoco_sim::v1::OperatorInput& input);

control::input_snapshot_t ResolveInput(const mujoco_sim::adapter_sdk::OperatorInputClient* operator_client,
                                       std::chrono::milliseconds max_age, const InputScript& script,
                                       std::uint64_t tick_id);

}  // namespace controller::sim
