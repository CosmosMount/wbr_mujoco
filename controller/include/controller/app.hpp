#pragma once

#include "controller/config.hpp"

#include <atomic>

namespace controller
{

class controller_app
{
public:
    explicit controller_app(const app_config& cfg);
    ~controller_app();
    int run();
    void shutdown();

private:
    app_config cfg_;
    std::atomic<bool> running_{true};
};

}  // namespace controller
