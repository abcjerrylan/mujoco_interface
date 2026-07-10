#pragma once

#include "mujoco_interface/protocol/messages.hpp"

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace mujoco_interface::core
{

class client_registry
{
public:
    struct client
    {
        std::uint32_t client_id = 0;
        std::uint32_t session_id = 0;
        std::uint32_t motor_begin = 0;
        std::uint32_t motor_count = 0;
        bool active = false;
    };

    protocol::register_ack_message register_client(const protocol::register_message& request,
                                                 std::uint32_t epoch,
                                                 std::uint32_t num_motors,
                                                 double timestep,
                                                 std::string& error);

    void clear();

    [[nodiscard]] const client* find(std::uint32_t client_id) const;
    [[nodiscard]] std::vector<client> active_clients() const;
    [[nodiscard]] bool owns_motor(const client& client, std::uint32_t motor_index) const;

private:
    bool overlaps(const client& lhs, const client& rhs) const;
    mutable std::mutex mutex_;
    std::unordered_map<std::uint32_t, client> clients_;
};

}  // namespace mujoco_interface::core
