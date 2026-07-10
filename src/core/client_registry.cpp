#include "mujoco_interface/core/client_registry.hpp"

#include <algorithm>

namespace mujoco_interface::core
{

protocol::register_ack_message client_registry::register_client(const protocol::register_message& request,
                                                                std::uint32_t epoch,
                                                                std::uint32_t num_motors,
                                                                double timestep,
                                                                std::string& error)
{
    const std::lock_guard<std::mutex> lock(mutex_);
    protocol::register_ack_message ack{};
    ack.sync.epoch = epoch;
    ack.session_id = request.session_id;
    ack.num_motors = num_motors;
    ack.timestep = timestep;

    if (request.client_id == 0)
    {
        error = "client_id must be non-zero";
        ack.accepted = false;
        return ack;
    }

    if (request.motor_count == 0 || request.motor_begin + request.motor_count > num_motors)
    {
        error = "invalid motor ownership range";
        ack.accepted = false;
        return ack;
    }

    client candidate{};
    candidate.client_id = request.client_id;
    candidate.session_id = request.session_id;
    candidate.motor_begin = request.motor_begin;
    candidate.motor_count = request.motor_count;
    candidate.active = true;

    for (const auto& [id, existing] : clients_)
    {
        if (!existing.active || id == candidate.client_id)
        {
            continue;
        }
        if (existing.session_id == candidate.session_id)
        {
            continue;
        }
        if (overlaps(existing, candidate))
        {
            error = "motor ownership overlaps client " + std::to_string(id);
            ack.accepted = false;
            return ack;
        }
    }

    clients_[candidate.client_id] = candidate;
    ack.accepted = true;
    return ack;
}

void client_registry::clear()
{
    const std::lock_guard<std::mutex> lock(mutex_);
    clients_.clear();
}

const client_registry::client* client_registry::find(std::uint32_t client_id) const
{
    const std::lock_guard<std::mutex> lock(mutex_);
    const auto it = clients_.find(client_id);
    if (it == clients_.end() || !it->second.active)
    {
        return nullptr;
    }
    return &it->second;
}

std::vector<client_registry::client> client_registry::active_clients() const
{
    const std::lock_guard<std::mutex> lock(mutex_);
    std::vector<client> out;
    out.reserve(clients_.size());
    for (const auto& [id, client] : clients_)
    {
        if (client.active)
        {
            out.push_back(client);
            (void)id;
        }
    }
    return out;
}

bool client_registry::owns_motor(const client& client, std::uint32_t motor_index) const
{
    return motor_index >= client.motor_begin && motor_index < client.motor_begin + client.motor_count;
}

bool client_registry::overlaps(const client& lhs, const client& rhs) const
{
    const auto lhs_begin = lhs.motor_begin;
    const auto lhs_end = lhs.motor_begin + lhs.motor_count;
    const auto rhs_begin = rhs.motor_begin;
    const auto rhs_end = rhs.motor_begin + rhs.motor_count;
    return lhs_begin < rhs_end && rhs_begin < lhs_end;
}

}  // namespace mujoco_interface::core
