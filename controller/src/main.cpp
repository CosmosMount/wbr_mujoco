#include "controller/app.hpp"
#include "controller/config.hpp"

#include <csignal>
#include <cstdio>

namespace
{

controller::controller_app* g_app = nullptr;

void handle_signal(int)
{
    if (g_app != nullptr)
    {
        g_app->shutdown();
    }
}

}  // namespace

int main(int argc, char** argv)
{
    const controller::app_config cfg = controller::load_config(argc, argv);

    controller::controller_app app(cfg);
    g_app = &app;

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    std::printf("ctrl started (server=%s, controller_id=%s, imu_mode=%d",
                cfg.adapter.server_name.c_str(), cfg.adapter.controller_id.c_str(),
                static_cast<int>(cfg.pipeline.imu_mode));
    if (cfg.logger.stdout_block && cfg.logger.hz > 0.0f)
    {
        std::printf(", log=%.1fHz", cfg.logger.hz);
    }
    std::printf("). Focus sim window for keyboard. Ctrl+C to exit.\n");
    return app.run();
}
