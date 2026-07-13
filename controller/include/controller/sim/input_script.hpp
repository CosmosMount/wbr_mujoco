#pragma once

#include <cstdint>
#include <vector>

#include "control/msgs.hpp"

namespace controller::sim
{

struct InputInterval
{
    std::uint64_t begin_tick = 0;
    std::uint64_t end_tick = 0;
    control::input_snapshot_t input{};
};

class InputScript
{
public:
    InputScript() = default;
    explicit InputScript(std::vector<InputInterval> intervals);

    control::input_snapshot_t InputAt(std::uint64_t tick_id) const;
    const std::vector<InputInterval>& intervals() const { return intervals_; }

private:
    std::vector<InputInterval> intervals_;
};

}  // namespace controller::sim
