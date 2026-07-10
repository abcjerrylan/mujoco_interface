#pragma once

#include "mujoco_interface/core/client_registry.hpp"
#include "mujoco_interface/protocol/messages.hpp"
#include "mujoco_interface/types.hpp"

#include <cstdint>
#include <vector>

namespace mujoco_interface::core
{

class command_arbiter
{
public:
    robot::command merge(const std::vector<protocol::command_envelope>& commits,
                         const client_registry& registry,
                         std::uint32_t num_motors) const;

    static robot::command zero_command(std::uint32_t num_motors);
};

}  // namespace mujoco_interface::core
