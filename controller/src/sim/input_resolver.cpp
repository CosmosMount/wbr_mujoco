#include "controller/sim/input_resolver.hpp"

namespace controller::sim
{

control::input_snapshot_t ToInputSnapshot(const mujoco_sim::v1::OperatorInput& input)
{
    control::input_snapshot_t snap{};
    snap.w = input.w();
    snap.s = input.s();
    snap.a = input.a();
    snap.d = input.d();
    snap.q = input.q();
    snap.e = input.e();
    snap.space = input.space();
    snap.r = input.r();
    snap.f = input.f();
    return snap;
}

control::input_snapshot_t ResolveInput(const mujoco_sim::adapter_sdk::OperatorInputClient* operator_client,
                                       std::chrono::milliseconds max_age, const InputScript& script,
                                       std::uint64_t tick_id)
{
    if (operator_client != nullptr)
    {
        const auto latest = operator_client->Latest(max_age);
        if (latest.has_value())
        {
            return ToInputSnapshot(latest->input);
        }
    }
    return script.InputAt(tick_id);
}

}  // namespace controller::sim
