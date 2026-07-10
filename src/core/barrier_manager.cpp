#include "mujoco_interface/core/barrier_manager.hpp"

#include <algorithm>
#include <thread>

namespace mujoco_interface::core
{

void barrier_manager::begin_tick(std::uint64_t tick_id, std::uint32_t epoch, const client_registry& registry)
{
    const std::lock_guard<std::mutex> lock(mutex_);
    tick_id_ = tick_id;
    epoch_ = epoch;
    commits_.clear();
    expected_clients_.clear();

    for (const auto& client : registry.active_clients())
    {
        expected_clients_.push_back(client.client_id);
    }
}

void barrier_manager::ensure_clients(const client_registry& registry)
{
    const std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& client : registry.active_clients())
    {
        if (std::find(expected_clients_.begin(), expected_clients_.end(), client.client_id) ==
            expected_clients_.end())
        {
            expected_clients_.push_back(client.client_id);
        }
    }
}

bool barrier_manager::submit_commit(const protocol::command_envelope& commit,
                                    const client_registry& registry,
                                    std::string& error)
{
    const std::lock_guard<std::mutex> lock(mutex_);

    if (commit.sync.tick_id != tick_id_ || commit.sync.epoch != epoch_)
    {
        error = "commit tick/epoch mismatch";
        return false;
    }

    const client_registry::client* client = registry.find(commit.sync.client_id);
    if (client == nullptr)
    {
        error = "unknown or inactive client";
        return false;
    }

    if (client->session_id != commit.sync.session_id)
    {
        error = "session mismatch";
        return false;
    }

    commits_[commit.sync.client_id] = commit;
    return true;
}

barrier_manager::wait_result barrier_manager::wait(std::chrono::microseconds timeout)
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline)
    {
        {
            const std::lock_guard<std::mutex> lock(mutex_);
            if (expected_clients_.empty())
            {
                return wait_result::ready;
            }

            bool all_ready = true;
            for (const auto client_id : expected_clients_)
            {
                if (commits_.find(client_id) == commits_.end())
                {
                    all_ready = false;
                    break;
                }
            }
            if (all_ready)
            {
                return wait_result::ready;
            }
        }

        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    return wait_result::timeout;
}

bool barrier_manager::all_ready() const
{
    const std::lock_guard<std::mutex> lock(mutex_);
    if (expected_clients_.empty())
    {
        return true;
    }

    for (const auto client_id : expected_clients_)
    {
        if (commits_.find(client_id) == commits_.end())
        {
            return false;
        }
    }
    return true;
}

std::vector<protocol::command_envelope> barrier_manager::commits() const
{
    const std::lock_guard<std::mutex> lock(mutex_);
    std::vector<protocol::command_envelope> out;
    out.reserve(commits_.size());
    for (const auto& [id, commit] : commits_)
    {
        out.push_back(commit);
        (void)id;
    }
    return out;
}

void barrier_manager::clear()
{
    const std::lock_guard<std::mutex> lock(mutex_);
    commits_.clear();
    expected_clients_.clear();
    tick_id_ = 0;
    epoch_ = 0;
}

}  // namespace mujoco_interface::core
