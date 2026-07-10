#include "mujoco_interface/core/barrier_manager.hpp"
#include "mujoco_interface/core/client_registry.hpp"
#include "mujoco_interface/core/command_arbiter.hpp"

#include <cassert>
#include <chrono>
#include <iostream>

namespace
{

using namespace mujoco_interface;

void test_client_registry()
{
    core::client_registry registry;
    std::string error;

    protocol::register_message request{1, 10, 0, 3};
    assert(registry.register_client(request, 1, 6, 0.001, error).accepted);

    protocol::register_message overlap{2, 11, 2, 2};
    assert(!registry.register_client(overlap, 1, 6, 0.001, error).accepted);
}

void test_command_arbiter()
{
    core::client_registry registry;
    std::string error;

    assert(registry.register_client({1, 1, 0, 3}, 1, 6, 0.001, error).accepted);
    assert(registry.register_client({2, 2, 3, 3}, 1, 6, 0.001, error).accepted);

    protocol::command_envelope commit_a{};
    commit_a.sync.client_id = 1;
    commit_a.body.motors[0].tau = 1.0f;

    protocol::command_envelope commit_b{};
    commit_b.sync.client_id = 2;
    commit_b.body.motors[5].tau = 2.0f;

    const robot::command merged =
        core::command_arbiter{}.merge({commit_a, commit_b}, registry, 6);
    assert(merged.motors[0].tau == 1.0f);
    assert(merged.motors[5].tau == 2.0f);
}

void test_barrier()
{
    core::client_registry registry;
    std::string error;
    assert(registry.register_client({1, 42, 0, 2}, 1, 2, 0.001, error).accepted);

    core::barrier_manager barrier;
    barrier.begin_tick(1, 1, registry);

    protocol::command_envelope commit{};
    commit.sync.tick_id = 1;
    commit.sync.epoch = 1;
    commit.sync.session_id = 42;
    commit.sync.client_id = 1;
    commit.body.num_motors = 2;
    assert(barrier.submit_commit(commit, registry, error));

    assert(barrier.wait(std::chrono::milliseconds(10)) == core::barrier_manager::wait_result::ready);
    assert(barrier.commits().size() == 1);
}

}  // namespace

int main()
{
    test_client_registry();
    test_command_arbiter();
    test_barrier();
    std::cout << "all core tests passed\n";
    return 0;
}
