#include "mujoco_interface/core/simulation.hpp"

#include <mujoco/mujoco.h>

#include <cstdio>
#include <thread>

namespace mujoco_interface::core
{

simulation::simulation(simulation_config config) : config_(std::move(config)) {}

bool simulation::init(const std::string& config_path, const std::string& scene_path, transport::server& transport,
                      std::string& error)
{
    if (!models_.load_scene(scene_path, error) || !models_.bind_robot(config_path, error))
    {
        return false;
    }

    transport::callbacks cb{};
    cb.on_register = [this, &transport](const protocol::register_message& request)
    {
        std::string register_error;
        protocol::register_ack_message ack = registry_.register_client(
            request, clock_.epoch(), models_.robot().num_motors(), models_.robot().sim_timestep(), register_error);
        ack.sync.tick_id = clock_.tick_id();
        ack.sync.client_id = protocol::k_sim_client_id;
        if (ack.accepted)
        {
            static std::uint32_t last_logged_client = 0;
            if (request.client_id != last_logged_client)
            {
                last_logged_client = request.client_id;
                std::fprintf(stderr, "sim: controller registered (client=%u epoch=%u)\n", request.client_id,
                             clock_.epoch());
            }
        }
        transport.publish_register_ack(ack);
    };
    cb.on_commit = [this](const protocol::command_envelope& commit)
    {
        std::string commit_error;
        barrier_.submit_commit(commit, registry_, commit_error);
    };

    if (!transport.start(config_.topic_namespace, cb, error))
    {
        return false;
    }

    clock_.reset_episode(models_.data()->time);
    clock_.bump_epoch();
    registry_.clear();
    barrier_.clear();
    last_command_ = command_arbiter::zero_command(models_.robot().num_motors());
    initialized_ = true;
    pending_reset_ = false;
    return true;
}

void simulation::shutdown(transport::server& transport)
{
    transport.stop();
    initialized_ = false;
}

void simulation::request_reset()
{
    pending_reset_ = true;
}

void simulation::run_tick_cycle(transport::server& transport)
{
    if (pending_reset_)
    {
        models_.reset_home();
        clock_.reset_episode(models_.data()->time);
        clock_.bump_epoch();
        registry_.clear();
        barrier_.clear();
        last_command_ = command_arbiter::zero_command(models_.robot().num_motors());
        pending_reset_ = false;
    }

    const std::uint64_t next_tick = clock_.tick_id() + 1;
    barrier_.begin_tick(next_tick, clock_.epoch(), registry_);

    protocol::tick_message tick{};
    tick.sync.tick_id = next_tick;
    tick.sync.epoch = clock_.epoch();
    tick.sync.client_id = protocol::k_sim_client_id;
    tick.sim_time = models_.data()->time;
    tick.commit_deadline_us = static_cast<std::uint64_t>(config_.commit_timeout.count());
    transport.publish_tick(tick);
    transport.poll();

    const auto deadline = std::chrono::steady_clock::now() + config_.commit_timeout;
    while (std::chrono::steady_clock::now() < deadline)
    {
        transport.poll();
        barrier_.ensure_clients(registry_);
        if (barrier_.all_ready())
        {
            break;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(50));
    }

    auto commits = barrier_.commits();
    if (commits.empty() && registry_.active_clients().empty())
    {
        static bool warned = false;
        if (!warned)
        {
            warned = true;
            std::fprintf(stderr, "sim: no registered controller; motor commands ignored\n");
        }
    }

    robot::command merged = last_command_;
    if (!commits.empty())
    {
        merged = arbiter_.merge(commits, registry_, models_.robot().num_motors());
        last_command_ = merged;
    }
    models_.robot().write_command(merged);
    mj_step(models_.model(), models_.data());

    const double timestep = models_.robot().sim_timestep();
    clock_.advance_tick(timestep > 0.0 ? timestep : models_.model()->opt.timestep);

    protocol::state_envelope envelope{};
    envelope.sync.tick_id = clock_.tick_id();
    envelope.sync.epoch = clock_.epoch();
    envelope.sync.client_id = protocol::k_sim_client_id;
    models_.robot().read_state(envelope.body);
    transport.publish_state(envelope);
    transport.poll();
}

bool simulation::step(transport::server& transport, std::string& error)
{
    if (!initialized_)
    {
        error = "simulation not initialized";
        return false;
    }
    run_tick_cycle(transport);
    return true;
}

}  // namespace mujoco_interface::core
