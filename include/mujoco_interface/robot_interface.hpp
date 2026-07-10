#pragma once

#include "mujoco_interface/robot_config.hpp"
#include "mujoco_interface/types.hpp"

#include <mujoco/mujoco.h>

#include <cstdint>
#include <string>
#include <vector>

namespace mujoco_interface::robot
{

class interface
{
public:
    bool load_config(const std::string& yaml_path, std::string& error);
    bool bind(mjModel* model, mjData* data, std::string& error);

    [[nodiscard]] bool is_bound() const { return model_ != nullptr && data_ != nullptr; }
    [[nodiscard]] const config& robot_config() const { return config_; }

    void read_state(state& out) const;
    void write_command(const command& cmd);
    [[nodiscard]] std::uint32_t num_motors() const
    {
        return static_cast<std::uint32_t>(motors_.size());
    }

    [[nodiscard]] double sim_timestep() const
    {
        return model_ != nullptr ? model_->opt.timestep : 0.0;
    }

    void reset_keyframe(const char* name = "home") const;

private:
    struct motor_binding
    {
        motor_mapping mapping;
        int actuator_id = -1;
        int pos_adr = -1;
        int pos_dim = 0;
        int vel_adr = -1;
        int vel_dim = 0;
        int tor_adr = -1;
        int tor_dim = 0;
    };

    void clear();
    bool bind_internal(mjModel* model, mjData* data, std::string& error);
    static bool resolve_sensor(mjModel* model, const char* name, int& adr, int& dim, std::string& error);

    config config_;
    std::string config_path_;
    mjModel* model_ = nullptr;
    mjData* data_ = nullptr;
    int imu_quat_adr_ = -1;
    int imu_gyro_adr_ = -1;
    int imu_accel_adr_ = -1;
    std::vector<motor_binding> motors_;
};

}  // namespace mujoco_interface::robot
