#include "mujoco_interface/core/command_arbiter.hpp"

#include <algorithm>
#include <cstring>

namespace mujoco_interface::core
{

robot::command command_arbiter::zero_command(std::uint32_t num_motors)
{
    robot::command cmd{};
    cmd.num_motors = num_motors;
    return cmd;
}

robot::command command_arbiter::merge(const std::vector<protocol::command_envelope>& commits,
                                      const client_registry& registry,
                                      std::uint32_t num_motors) const
{
    robot::command merged = zero_command(num_motors);

    for (const auto& commit : commits)
    {
        const client_registry::client* client = registry.find(commit.sync.client_id);
        if (client == nullptr)
        {
            continue;
        }

        const std::uint32_t count =
            std::min(commit.body.num_motors, num_motors);
        for (std::uint32_t i = 0; i < count; ++i)
        {
            if (!registry.owns_motor(*client, i))
            {
                continue;
            }
            merged.motors[i] = commit.body.motors[i];
        }
        merged.num_motors = std::max(merged.num_motors, count);
    }

    return merged;
}

}  // namespace mujoco_interface::core
