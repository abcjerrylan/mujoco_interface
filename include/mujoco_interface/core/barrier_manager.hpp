#pragma once

#include "mujoco_interface/core/client_registry.hpp"
#include "mujoco_interface/protocol/messages.hpp"

#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <unordered_map>
#include <vector>

namespace mujoco_interface::core
{

class barrier_manager
{
public:
    void begin_tick(std::uint64_t tick_id, std::uint32_t epoch, const client_registry& registry);

    void ensure_clients(const client_registry& registry);

    bool submit_commit(const protocol::command_envelope& commit,
                       const client_registry& registry,
                       std::string& error);

    enum class wait_result
    {
        ready,
        timeout,
    };

    wait_result wait(std::chrono::microseconds timeout);

    [[nodiscard]] bool all_ready() const;

    [[nodiscard]] std::vector<protocol::command_envelope> commits() const;
    void clear();

private:
    mutable std::mutex mutex_;
    std::uint64_t tick_id_ = 0;
    std::uint32_t epoch_ = 0;
    std::unordered_map<std::uint32_t, protocol::command_envelope> commits_;
    std::vector<std::uint32_t> expected_clients_;
};

}  // namespace mujoco_interface::core
