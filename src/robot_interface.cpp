#include "mujoco_interface/robot_interface.hpp"

#include <mujoco/mujoco.h>

#include <cmath>
#include <cstring>

namespace mujoco_interface::robot
{

bool interface::load_config(const std::string& yaml_path, std::string& error)
{
    config_path_ = yaml_path;
    return robot::load_config(yaml_path, config_, error);
}

void interface::clear()
{
    model_ = nullptr;
    data_ = nullptr;
    motors_.clear();
    imu_quat_adr_ = -1;
    imu_gyro_adr_ = -1;
    imu_accel_adr_ = -1;
}

bool interface::resolve_sensor(mjModel* model, const char* name, int& adr, int& dim, std::string& error)
{
    const int sensor_id = mj_name2id(model, mjOBJ_SENSOR, name);
    if (sensor_id < 0)
    {
        error = std::string("missing sensor: ") + name;
        return false;
    }
    adr = model->sensor_adr[sensor_id];
    dim = model->sensor_dim[sensor_id];
    return true;
}

bool interface::bind_internal(mjModel* model, mjData* data, std::string& error)
{
    model_ = model;
    data_ = data;
    motors_.clear();

    int dim = 0;
    if (!resolve_sensor(model_, config_.imu.quat.c_str(), imu_quat_adr_, dim, error))
    {
        return false;
    }
    if (!resolve_sensor(model_, config_.imu.gyro.c_str(), imu_gyro_adr_, dim, error))
    {
        return false;
    }
    if (!resolve_sensor(model_, config_.imu.accel.c_str(), imu_accel_adr_, dim, error))
    {
        return false;
    }

    motors_.reserve(config_.motors.size());
    for (const auto& mapping : config_.motors)
    {
        motor_binding binding;
        binding.mapping = mapping;

        binding.actuator_id = mj_name2id(model_, mjOBJ_ACTUATOR, mapping.actuator.c_str());
        if (binding.actuator_id < 0)
        {
            error = "missing actuator: " + mapping.actuator;
            return false;
        }

        if (!resolve_sensor(model_, mapping.pos_sensor.c_str(), binding.pos_adr, binding.pos_dim, error) ||
            !resolve_sensor(model_, mapping.vel_sensor.c_str(), binding.vel_adr, binding.vel_dim, error) ||
            !resolve_sensor(model_, mapping.tor_sensor.c_str(), binding.tor_adr, binding.tor_dim, error))
        {
            return false;
        }

        motors_.push_back(binding);
    }

    if (static_cast<int>(motors_.size()) != model_->nu)
    {
        error = "motor count (" + std::to_string(motors_.size()) + ") != nu (" + std::to_string(model_->nu) + ")";
        return false;
    }

    if (config_.timestep > 0.0)
    {
        model_->opt.timestep = config_.timestep;
    }

    return true;
}

bool interface::bind(mjModel* model, mjData* data, std::string& error)
{
    clear();
    return bind_internal(model, data, error);
}

void interface::read_state(state& out) const
{
    std::memset(&out, 0, sizeof(out));
    out.time = data_->time;
    out.num_motors = static_cast<std::uint32_t>(motors_.size());

    if (imu_quat_adr_ >= 0)
    {
        for (int i = 0; i < 4; ++i)
        {
            out.imu.quat[i] = static_cast<float>(data_->sensordata[imu_quat_adr_ + i]);
        }
    }
    if (imu_gyro_adr_ >= 0)
    {
        for (int i = 0; i < 3; ++i)
        {
            out.imu.gyro[i] = static_cast<float>(data_->sensordata[imu_gyro_adr_ + i]);
        }
    }
    if (imu_accel_adr_ >= 0)
    {
        for (int i = 0; i < 3; ++i)
        {
            out.imu.accel[i] = static_cast<float>(data_->sensordata[imu_accel_adr_ + i]);
        }
    }

    for (std::size_t i = 0; i < motors_.size(); ++i)
    {
        const auto& motor = motors_[i];
        out.motors[i].q = static_cast<float>(data_->sensordata[motor.pos_adr] * motor.mapping.sign.pos);
        out.motors[i].dq = static_cast<float>(data_->sensordata[motor.vel_adr] * motor.mapping.sign.vel);
        out.motors[i].tau_est = static_cast<float>(data_->sensordata[motor.tor_adr] * motor.mapping.sign.cmd);
    }
}

void interface::write_command(const command& cmd)
{
    const std::size_t count = motors_.size();
    const std::size_t n = std::min<std::size_t>(count, cmd.num_motors);

    for (std::size_t i = 0; i < n; ++i)
    {
        const auto& binding = motors_[i];
        const auto& motor_cmd = cmd.motors[i];
        const double q = data_->sensordata[binding.pos_adr];
        const double dq = data_->sensordata[binding.vel_adr];

        double torque = motor_cmd.tau;
        if (motor_cmd.mode == mode_pd)
        {
            torque += motor_cmd.kp * (motor_cmd.q - q) + motor_cmd.kd * (motor_cmd.dq - dq);
        }

        data_->ctrl[binding.actuator_id] = binding.mapping.sign.cmd * torque;
    }

    for (std::size_t i = n; i < count; ++i)
    {
        data_->ctrl[motors_[i].actuator_id] = 0.0;
    }
}

void interface::reset_keyframe(const char* name) const
{
    if (!is_bound())
    {
        return;
    }

    const int key_id = mj_name2id(model_, mjOBJ_KEY, name);
    if (key_id >= 0)
    {
        mj_resetDataKeyframe(model_, data_, key_id);
        mj_forward(model_, data_);
    }
}

}  // namespace mujoco_interface::robot
