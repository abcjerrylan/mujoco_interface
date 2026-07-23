#pragma once

#include "mujoco_interface/core/barrier_manager.hpp"
#include "mujoco_interface/core/clock.hpp"
#include "mujoco_interface/core/client_registry.hpp"
#include "mujoco_interface/core/command_arbiter.hpp"
#include "mujoco_interface/core/model_manager.hpp"
#include "mujoco_interface/protocol/messages.hpp"
#include "mujoco_interface/transport/ecal.hpp"

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>

namespace mujoco_interface::core
{

inline constexpr std::chrono::microseconds k_default_commit_timeout{5000};

struct simulation_config
{
    std::string topic_namespace = "mujoco_sim";
    std::chrono::microseconds commit_timeout{k_default_commit_timeout};
    std::uint64_t command_hold_ticks = 5;
};

class simulation
{
public:
    explicit simulation(simulation_config config = {});

    bool init(const std::string& config_path, const std::string& scene_path, transport::server& transport,
              std::string& error);
    void shutdown(transport::server& transport);
    bool step(transport::server& transport, std::string& error);
    void request_reset();
    [[nodiscard]] mjModel* copy_model() const;
    [[nodiscard]] mjData* copy_data_for(mjModel* model) const;
    bool copy_data_to(mjModel* model, mjData* data, std::string& error) const;
    [[nodiscard]] double simulation_time() const;
    [[nodiscard]] double timestep() const;

    [[nodiscard]] model_manager& models() { return models_; }
    [[nodiscard]] const model_manager& models() const { return models_; }

private:
    void run_tick_cycle(transport::server& transport);

    simulation_config config_;
    model_manager models_;
    clock clock_;
    client_registry registry_;
    barrier_manager barrier_;
    command_arbiter arbiter_;
    mutable std::mutex mutex_;
    bool initialized_ = false;
    bool pending_reset_ = false;
    bool warned_no_clients_ = false;
    bool controller_has_registered_ = false;
    std::uint64_t consecutive_missing_commits_ = 0;
    std::uint64_t rejected_commits_ = 0;
    robot::command last_command_{};
};

}  // namespace mujoco_interface::core
