#pragma once

#include "mujoco_interface/input.hpp"

#include <functional>
#include <memory>

struct GLFWwindow;

namespace mujoco_interface::input
{

// GLFW input capture for sim viewer. Implemented in mujoco_interface.
class hub
{
public:
    using key_handler = std::function<void(key k, int scancode, key_action action, int mods)>;

    static hub& instance();

    void attach(GLFWwindow* window);
    void set_key_handler(key_handler handler);

    // Captured keys are recorded in take_snapshot() but not forwarded to the MuJoCo viewer.
    void capture_key(key k);
    void release_key(key k);
    void clear_captured_keys();
    void capture_wasd_keys();
    void capture_arrow_keys();
    void capture_control_keys();

    [[nodiscard]] snapshot take_snapshot();

private:
    hub() = default;

    struct impl;
    std::unique_ptr<impl> impl_;
};

}  // namespace mujoco_interface::input
