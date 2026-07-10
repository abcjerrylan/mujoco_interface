#include "mujoco_interface/core/clock.hpp"

namespace mujoco_interface::core
{

void clock::reset_episode(double sim_time)
{
    sim_time_ = sim_time;
    episode_start_sim_time_ = sim_time;
    episode_time_ = 0.0;
    tick_id_ = 0;
}

void clock::bump_epoch()
{
    ++epoch_;
    tick_id_ = 0;
}

void clock::advance_tick(double timestep)
{
    ++tick_id_;
    sim_time_ += timestep;
    episode_time_ = sim_time_ - episode_start_sim_time_;
}

}  // namespace mujoco_interface::core
