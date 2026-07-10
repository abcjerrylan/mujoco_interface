#include "mujoco_interface/robot_config.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>

namespace mujoco_interface::robot
{

namespace
{

motor_sign parse_sign(const YAML::Node& node)
{
    // NOTE: sign mapping is intentionally disabled.
    // All sign conventions are handled by controller-side `reverse` only.
    (void)node;
    return motor_sign{};
}

motor_mapping parse_motor(const YAML::Node& node)
{
    motor_mapping motor;
    motor.name = node["name"].as<std::string>();
    motor.actuator = node["actuator"].as<std::string>();
    motor.pos_sensor = node["pos"].as<std::string>();
    motor.vel_sensor = node["vel"].as<std::string>();
    motor.tor_sensor = node["tor"].as<std::string>();
    motor.sign = parse_sign(node["sign"]);
    return motor;
}

}  // namespace

bool load_config(const std::string& path, config& out, std::string& error)
{
    try
    {
        const YAML::Node root = YAML::LoadFile(path);
        if (root["schema_version"])
        {
            out.schema_version = root["schema_version"].as<int>();
        }
        out.robot = root["robot"].as<std::string>();
        out.scene = root["scene"].as<std::string>();

        if (root["timestep"])
        {
            out.timestep = root["timestep"].as<double>();
        }
        const YAML::Node imu = root["imu"];
        out.imu.quat = imu["quat"].as<std::string>();
        out.imu.gyro = imu["gyro"].as<std::string>();
        out.imu.accel = imu["accel"].as<std::string>();

        out.motors.clear();
        for (const auto& motor_node : root["motors"])
        {
            out.motors.push_back(parse_motor(motor_node));
        }

        if (out.motors.empty())
        {
            error = "config has no motors";
            return false;
        }

        if (out.schema_version != 1)
        {
            error = "unsupported schema_version: " + std::to_string(out.schema_version);
            return false;
        }

        return true;
    }
    catch (const std::exception& ex)
    {
        error = ex.what();
        return false;
    }
}

std::string resolve_path(const std::string& config_path, const std::string& relative_path)
{
    const std::filesystem::path base = std::filesystem::absolute(config_path).parent_path();
    return std::filesystem::weakly_canonical(base / relative_path).string();
}

}  // namespace mujoco_interface::robot
