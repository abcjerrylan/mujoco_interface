#pragma once

#include "mujoco_interface/robot_interface.hpp"

#include <mujoco/mujoco.h>

#include <string>

namespace mujoco_interface::core
{

class model_manager
{
public:
    model_manager() = default;
    ~model_manager();

    model_manager(const model_manager&) = delete;
    model_manager& operator=(const model_manager&) = delete;

    bool load_scene(const std::string& scene_path, std::string& error);
    bool bind_robot(const std::string& config_path, std::string& error);
    void reset_home();

    [[nodiscard]] mjModel* model() const { return model_; }
    [[nodiscard]] mjData* data() const { return data_; }
    [[nodiscard]] robot::interface& robot() { return robot_; }
    [[nodiscard]] const robot::interface& robot() const { return robot_; }
    [[nodiscard]] const std::string& config_path() const { return config_path_; }

private:
    void destroy();

    mjModel* model_ = nullptr;
    mjData* data_ = nullptr;
    robot::interface robot_;
    std::string config_path_;
};

}  // namespace mujoco_interface::core
