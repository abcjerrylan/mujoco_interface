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
        protocol::register_ack_message ack{};
        {
            const std::lock_guard<std::mutex> lock(mutex_);
            ack = registry_.register_client(
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
        }
        transport.publish_register_ack(ack);
    };
    cb.on_commit = [this](const protocol::command_envelope& commit)
    {
        std::string commit_error;
        const std::lock_guard<std::mutex> lock(mutex_);
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
    const std::lock_guard<std::mutex> lock(mutex_);
    pending_reset_ = true;
}

void simulation::run_tick_cycle(transport::server& transport)
{
    {
        const std::lock_guard<std::mutex> lock(mutex_);
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
    }

    protocol::tick_message tick{};
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        const std::uint64_t next_tick = clock_.tick_id() + 1;
        barrier_.begin_tick(next_tick, clock_.epoch(), registry_);
        tick.sync.tick_id = next_tick;
        tick.sync.epoch = clock_.epoch();
        tick.sync.client_id = protocol::k_sim_client_id;
        tick.sim_time = models_.data()->time;
        tick.commit_deadline_us = static_cast<std::uint64_t>(config_.commit_timeout.count());
    }
    transport.publish_tick(tick);
    transport.poll();

    const auto deadline = std::chrono::steady_clock::now() + config_.commit_timeout;
    while (std::chrono::steady_clock::now() < deadline)
    {
        transport.poll();
        {
            const std::lock_guard<std::mutex> lock(mutex_);
            barrier_.ensure_clients(registry_);
            if (barrier_.all_ready())
            {
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::microseconds(50));
    }

    auto commits = barrier_.commits();
    bool no_active_clients = false;
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        no_active_clients = registry_.active_clients().empty();
    }
    if (commits.empty())
    {
        static bool warned = false;
        if (no_active_clients && !warned)
        {
            warned = true;
            std::fprintf(stderr, "sim: no controller command received; holding home pose\n");
        }
    }

    robot::command merged = last_command_;
    if (!commits.empty())
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        merged = arbiter_.merge(commits, registry_, models_.robot().num_motors());
        last_command_ = merged;
    }
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        if (commits.empty())
        {
            const double timestep = models_.robot().sim_timestep() > 0.0 ? models_.robot().sim_timestep()
                                                                         : models_.model()->opt.timestep;
            models_.reset_home_at_time(models_.data()->time + timestep);
            last_command_ = command_arbiter::zero_command(models_.robot().num_motors());
        }
        else
        {
            models_.robot().write_command(merged);
            mj_step(models_.model(), models_.data());
        }
    }

    protocol::state_envelope envelope{};
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        const double timestep = models_.robot().sim_timestep();
        clock_.advance_tick(timestep > 0.0 ? timestep : models_.model()->opt.timestep);
        envelope.sync.tick_id = clock_.tick_id();
        envelope.sync.epoch = clock_.epoch();
        envelope.sync.client_id = protocol::k_sim_client_id;
        models_.robot().read_state(envelope.body);
    }
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

mjModel* simulation::copy_model() const
{
    const std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_ || models_.model() == nullptr)
    {
        return nullptr;
    }
    return mj_copyModel(nullptr, models_.model());
}

mjData* simulation::copy_data_for(mjModel* model) const
{
    const std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_ || model == nullptr || models_.data() == nullptr)
    {
        return nullptr;
    }
    return mj_copyData(nullptr, model, models_.data());
}

bool simulation::copy_data_to(mjModel* model, mjData* data, std::string& error) const
{
    const std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_ || model == nullptr || data == nullptr || models_.data() == nullptr)
    {
        error = "simulation snapshot target is not initialized";
        return false;
    }
    if (model->nbuffer != models_.model()->nbuffer)
    {
        error = "simulation snapshot model is incompatible";
        return false;
    }
    mj_copyData(data, model, models_.data());
    return true;
}

double simulation::simulation_time() const
{
    const std::lock_guard<std::mutex> lock(mutex_);
    return initialized_ && models_.data() != nullptr ? models_.data()->time : 0.0;
}

double simulation::timestep() const
{
    const std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_)
    {
        return 0.0;
    }
    const double robot_timestep = models_.robot().sim_timestep();
    if (robot_timestep > 0.0)
    {
        return robot_timestep;
    }
    return models_.model() != nullptr ? models_.model()->opt.timestep : 0.0;
}

}  // namespace mujoco_interface::core
