#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace mujoco_interface::robot
{

struct motor_sign
{
    double pos = 1.0;
    double vel = 1.0;
    double cmd = 1.0;
};

struct motor_mapping
{
    std::string name;
    std::string actuator;
    std::string pos_sensor;
    std::string vel_sensor;
    std::string tor_sensor;
    motor_sign sign{};
};

struct imu_mapping
{
    std::string quat;
    std::string gyro;
    std::string accel;
};

struct config
{
    int schema_version = 1;
    std::string robot;
    std::string scene;
    double timestep = 0.0;
    imu_mapping imu{};
    std::vector<motor_mapping> motors;
};

bool load_config(const std::string& path, config& out, std::string& error);

std::string resolve_path(const std::string& config_path, const std::string& relative_path);

}  // namespace mujoco_interface::robot
