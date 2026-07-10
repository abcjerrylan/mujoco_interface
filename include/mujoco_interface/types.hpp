#pragma once

#include <cstdint>

namespace mujoco_interface::robot
{

inline constexpr std::uint16_t interface_version = 1;
inline constexpr std::uint8_t max_motors = 32;

inline constexpr std::uint8_t mode_torque = 0;
inline constexpr std::uint8_t mode_pd = 1;

struct motor_state
{
    float q = 0.0f;
    float dq = 0.0f;
    float tau_est = 0.0f;
};

struct motor_command
{
    float q = 0.0f;
    float dq = 0.0f;
    float tau = 0.0f;
    float kp = 0.0f;
    float kd = 0.0f;
    std::uint8_t mode = mode_torque;
    std::uint8_t _pad[3] = {};
};

struct imu_state
{
    float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // w, x, y, z
    float gyro[3] = {};
    float accel[3] = {};
};

struct state
{
    std::uint32_t num_motors = 0;
    double time = 0.0;
    imu_state imu{};
    motor_state motors[max_motors]{};
};

struct command
{
    std::uint32_t num_motors = 0;
    motor_command motors[max_motors]{};
};

}  // namespace mujoco_interface::robot
