#pragma once

#include <cstdint>

namespace mujoco_interface::core
{

class clock
{
public:
    [[nodiscard]] std::uint32_t epoch() const { return epoch_; }
    [[nodiscard]] std::uint64_t tick_id() const { return tick_id_; }
    [[nodiscard]] double sim_time() const { return sim_time_; }
    [[nodiscard]] double episode_time() const { return episode_time_; }

    void reset_episode(double sim_time);
    void bump_epoch();
    void advance_tick(double timestep);

private:
    std::uint32_t epoch_ = 1;
    std::uint64_t tick_id_ = 0;
    double sim_time_ = 0.0;
    double episode_time_ = 0.0;
    double episode_start_sim_time_ = 0.0;
};

}  // namespace mujoco_interface::core
