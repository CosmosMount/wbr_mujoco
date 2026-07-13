#include "controller/app.hpp"

#include "controller/sim/input_resolver.hpp"
#include "controller/sim/tick_controller.hpp"

#include <mujoco_sim/adapter_sdk/operator_input_client.hpp>
#include <mujoco_sim/adapter_sdk/sync_adapter_client.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <thread>

namespace controller
{

namespace
{

const char* FsmName(control::chassis_state state)
{
    switch (state)
    {
        case control::chassis_state::relax:
            return "relax";
        case control::chassis_state::recover:
            return "recover";
        case control::chassis_state::flatten:
            return "flatten";
        case control::chassis_state::neutral:
            return "neutral";
        case control::chassis_state::normal:
            return "normal";
        case control::chassis_state::offground:
            return "offground";
        case control::chassis_state::spin:
            return "spin";
        case control::chassis_state::gostair:
            return "gostair";
        case control::chassis_state::jump:
            return "jump";
    }
    return "unknown";
}

void PrintStateBlock(const sim::PipelineOutput& output)
{
    const auto& imu = output.imu;
    const auto& left = output.left;
    const auto& right = output.right;
    const auto& cmd = output.command;
    std::printf("[tick=%llu] imu quat=(%.3f %.3f %.3f %.3f) rpy_rad=(%.4f %.4f %.4f) gyro=(%.3f %.3f %.3f) "
                "accel=(%.3f %.3f %.3f)\n",
                static_cast<unsigned long long>(output.tick_index), imu.quaternion[0], imu.quaternion[1],
                imu.quaternion[2], imu.quaternion[3], imu.roll, imu.pitch, imu.yaw, imu.gyro_r, imu.gyro_p,
                imu.gyro_y, imu.accel[0], imu.accel[1], imu.accel[2]);
    std::printf("  left_link: len=%.4f phi=%.4f alpha=%.4f n=%.4f\n", left.length, left.phi, left.alpha,
                left.normal_force);
    std::printf("  right_link: len=%.4f phi=%.4f alpha=%.4f n=%.4f\n", right.length, right.phi, right.alpha,
                right.normal_force);
    std::printf("  state: fsm=%s cmd.len=%.4f cmd.v=%.4f move=%d odom=(x=%.4f v=%.4f az=%.4f)\n",
                FsmName(output.fsm_state), cmd.len, cmd.v, static_cast<int>(cmd.move), output.odometry.x,
                output.odometry.v, output.odometry.a_z);
    std::fflush(stdout);
}

}  // namespace

controller_app::controller_app(const app_config& cfg) : cfg_(cfg) {}

controller_app::~controller_app() = default;

void controller_app::shutdown()
{
    running_.store(false);
}

int controller_app::run()
{
    mujoco_sim::adapter_sdk::SyncAdapterClient client(sim::MakeAdapterOptions(cfg_.adapter));
    std::string error;
    if (!client.Start(&error))
    {
        std::fprintf(stderr, "adapter start failed: %s\n", error.c_str());
        if (error.find("RegisterController") != std::string::npos)
        {
            std::fprintf(stderr,
                         "hint: start mujoco-sim-server in another terminal first, e.g.\n"
                         "  source scripts/env.sh\n"
                         "  mujoco-sim-server --config config/robots/wbr_server.yaml\n");
        }
        // Skip C++/eCAL destructors: aborting in-flight RegisterController calls
        // races eCAL's DynamicThreadPool and can abort with
        // "Resource deadlock avoided".
        std::_Exit(1);
    }

    mujoco_sim::adapter_sdk::OperatorInputClient operator_client(
        mujoco_sim::adapter_sdk::OperatorInputClientOptions{cfg_.adapter.server_name});
    if (!operator_client.Start(&error))
    {
        std::fprintf(stderr, "operator input client start failed: %s\n", error.c_str());
        return 1;
    }

    sim::TickController controller(client.registration().layout(), cfg_.adapter.interfaces, cfg_.pipeline,
                                   cfg_.input_script);

    if (!client.Activate(&error))
    {
        std::fprintf(stderr, "adapter activate failed: %s\n", error.c_str());
        return 1;
    }

    const auto operator_max_age = std::chrono::milliseconds(cfg_.adapter.operator_input_max_age_ms);
    const std::uint64_t log_every =
        cfg_.log_every_ticks > 0
            ? cfg_.log_every_ticks
            : (cfg_.logger.stdout_block && cfg_.logger.hz > 0.0f
                   ? static_cast<std::uint64_t>(1.0f / cfg_.logger.hz / sim::kControlDtSeconds + 0.5f)
                   : 0);
    std::uint64_t handled_ticks = 0;
    std::uint64_t clamp_count = 0;
    int result = 0;

    while (running_.load() &&
           (cfg_.adapter.max_ticks == 0 || handled_ticks < cfg_.adapter.max_ticks))
    {
        mujoco_sim::v1::Tick tick;
        const auto wait_result =
            client.WaitForTick(std::chrono::milliseconds(100), &tick, &error);
        if (wait_result == mujoco_sim::adapter_sdk::TickWaitResult::kTimeout)
        {
            continue;
        }
        if (wait_result != mujoco_sim::adapter_sdk::TickWaitResult::kReceived)
        {
            std::fprintf(stderr, "%s\n",
                         error.empty() ? "adapter stopped while waiting for Tick" : error.c_str());
            result = 1;
            break;
        }

        try
        {
            const auto input =
                sim::ResolveInput(&operator_client, operator_max_age, cfg_.input_script, tick.header().tick_id());
            auto commands = controller.HandleTick(tick, input);
            if (!client.Commit(tick, commands, &error))
            {
                std::fprintf(stderr, "failed to Commit Tick: %s\n", error.c_str());
                result = 1;
                break;
            }

            if (tick.controller_state() == mujoco_sim::v1::ACTIVE)
            {
                ++handled_ticks;
            }
            clamp_count += controller.last_clamp_count();

            if (tick.controller_state() == mujoco_sim::v1::ACTIVE && log_every > 0 &&
                handled_ticks % log_every == 0 && controller.last_output().has_value())
            {
                PrintStateBlock(*controller.last_output());
            }
        }
        catch (const std::exception& ex)
        {
            std::fprintf(stderr, "%s\n", ex.what());
            result = 1;
            client.Stop();
            break;
        }
    }

    client.Stop();
    std::string unregister_error;
    if (!client.Unregister(&unregister_error))
    {
        std::fprintf(stderr, "adapter unregister failed: %s\n", unregister_error.c_str());
    }
    if (!client.lifecycle_warning().empty())
    {
        std::fprintf(stderr, "adapter lifecycle warning: %s\n", client.lifecycle_warning().c_str());
    }

    std::printf("ctrl summary ticks=%llu clamps=%llu result=%d\n",
                static_cast<unsigned long long>(handled_ticks),
                static_cast<unsigned long long>(clamp_count), result);
    std::fflush(stdout);
    operator_client.Stop();
    return result;
}

}  // namespace controller
