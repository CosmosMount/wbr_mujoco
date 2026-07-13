#include <cassert>
#include <cstdio>

#include "controller/sim/input_script.hpp"

int main()
{
    controller::sim::InputScript script({{1, 2, {.space = true}}});
    assert(!script.InputAt(0).space);
    assert(script.InputAt(1).space);
    assert(!script.InputAt(2).space);

    controller::sim::InputScript scripted({{10, 12, {.e = true}}});
    assert(scripted.InputAt(11).e);
    assert(!scripted.InputAt(12).e);

    std::printf("input_script_test ok\n");
    return 0;
}
