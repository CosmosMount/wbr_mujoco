#include "controller/sim/input_script.hpp"

#include <algorithm>
#include <iterator>
#include <stdexcept>
#include <string>
#include <utility>

namespace controller::sim {

InputScript::InputScript(std::vector<InputInterval> intervals)
    : intervals_(std::move(intervals)) {
  std::sort(intervals_.begin(), intervals_.end(),
            [](const InputInterval& lhs, const InputInterval& rhs) {
              if (lhs.begin_tick != rhs.begin_tick) {
                return lhs.begin_tick < rhs.begin_tick;
              }
              return lhs.end_tick < rhs.end_tick;
            });

  for (std::size_t i = 0; i < intervals_.size(); ++i) {
    const auto& interval = intervals_[i];
    if (interval.begin_tick >= interval.end_tick) {
      throw std::invalid_argument("input interval must satisfy begin_tick < end_tick");
    }
    if (i > 0 && interval.begin_tick < intervals_[i - 1].end_tick) {
      throw std::invalid_argument("input intervals must not overlap");
    }
  }
}

control::input_snapshot_t InputScript::InputAt(std::uint64_t tick_id) const {
  const auto next = std::upper_bound(
      intervals_.begin(), intervals_.end(), tick_id,
      [](std::uint64_t tick, const InputInterval& interval) {
        return tick < interval.begin_tick;
      });
  if (next == intervals_.begin()) {
    return {};
  }

  const auto& candidate = *std::prev(next);
  return tick_id < candidate.end_tick ? candidate.input
                                      : control::input_snapshot_t{};
}

}  // namespace controller::sim
