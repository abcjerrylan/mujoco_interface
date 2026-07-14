#include "mujoco_interface/core/model_manager.hpp"

#include <mujoco/mujoco.h>

#include <cstring>

namespace mujoco_interface::core
{

namespace
{

constexpr int k_error_length = 1024;

}  // namespace

model_manager::~model_manager()
{
    destroy();
}

void model_manager::destroy()
{
    if (data_ != nullptr)
    {
        mj_deleteData(data_);
        data_ = nullptr;
    }
    if (model_ != nullptr)
    {
        mj_deleteModel(model_);
        model_ = nullptr;
    }
}

bool model_manager::load_scene(const std::string& scene_path, std::string& error)
{
    destroy();

    char load_error[k_error_length] = "";
    model_ = mj_loadXML(scene_path.c_str(), nullptr, load_error, k_error_length);
    if (model_ == nullptr)
    {
        error = load_error[0] != '\0' ? load_error : "failed to load scene";
        return false;
    }

    data_ = mj_makeData(model_);
    if (data_ == nullptr)
    {
        error = "failed to allocate mjData";
        destroy();
        return false;
    }

    mj_forward(model_, data_);
    return true;
}

bool model_manager::bind_robot(const std::string& config_path, std::string& error)
{
    config_path_ = config_path;
    if (!robot_.load_config(config_path, error))
    {
        return false;
    }
    if (!robot_.bind(model_, data_, error))
    {
        return false;
    }
    reset_home();
    return true;
}

void model_manager::reset_home()
{
    robot_.reset_keyframe("home");
    if (model_ != nullptr && data_ != nullptr)
    {
        mju_zero(data_->ctrl, model_->nu);
        mj_forward(model_, data_);
    }
}

void model_manager::reset_home_at_time(double sim_time)
{
    reset_home();
    if (data_ != nullptr)
    {
        data_->time = sim_time;
        if (model_ != nullptr)
        {
            mj_forward(model_, data_);
        }
    }
}

}  // namespace mujoco_interface::core
