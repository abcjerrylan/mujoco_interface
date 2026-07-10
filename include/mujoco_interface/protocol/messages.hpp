#pragma once

#include "mujoco_interface/types.hpp"

#include <cstdint>

namespace mujoco_interface::protocol
{

inline constexpr std::uint32_t k_sim_client_id = 0;

struct sync_header
{
    std::uint64_t tick_id = 0;
    std::uint32_t epoch = 0;
    std::uint32_t session_id = 0;
    std::uint32_t client_id = 0;
};

struct state_envelope
{
    sync_header sync{};
    robot::state body{};
};

struct command_envelope
{
    sync_header sync{};
    robot::command body{};
};

struct tick_message
{
    sync_header sync{};
    double sim_time = 0.0;
    std::uint64_t commit_deadline_us = 0;
};

struct register_message
{
    std::uint32_t client_id = 0;
    std::uint32_t session_id = 0;
    std::uint32_t motor_begin = 0;
    std::uint32_t motor_count = 0;
};

struct register_ack_message
{
    sync_header sync{};
    std::uint32_t session_id = 0;
    std::uint32_t num_motors = 0;
    double timestep = 0.0;
    bool accepted = false;
};

struct input_message
{
    bool w = false;
    bool s = false;
    bool a = false;
    bool d = false;
    bool q = false;
    bool e = false;
    bool space = false;
    bool r = false;
    bool f = false;
};

}  // namespace mujoco_interface::protocol
