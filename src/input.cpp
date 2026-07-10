#include "mujoco_interface/input_hub.hpp"

#include <GLFW/glfw3.h>

#include <mutex>

namespace mujoco_interface::input
{
namespace
{

struct hub_backend
{
    std::mutex mutex;
    GLFWwindow* window = nullptr;
    GLFWwindow* attached_window = nullptr;

    double mouse_x = 0.0;
    double mouse_y = 0.0;
    double mouse_dx = 0.0;
    double mouse_dy = 0.0;
    double scroll_x = 0.0;
    double scroll_y = 0.0;
    bool mouse_left = false;
    bool mouse_right = false;
    bool mouse_middle = false;
    bool key_down[key_capacity]{};
    bool capture_mask[key_capacity]{};

    GLFWkeyfun prev_key = nullptr;
    GLFWmousebuttonfun prev_mouse_button = nullptr;
    GLFWcursorposfun prev_cursor_pos = nullptr;
    GLFWscrollfun prev_scroll = nullptr;

    hub::key_handler on_key;
};

hub_backend& hub_backend_ref()
{
    static hub_backend instance;
    return instance;
}

void set_key_down(int key, int action)
{
    auto& b = hub_backend_ref();
    if (!is_valid_key_code(key))
    {
        return;
    }

    if (action == GLFW_PRESS || action == GLFW_REPEAT)
    {
        b.key_down[key] = true;
    }
    else if (action == GLFW_RELEASE)
    {
        b.key_down[key] = false;
    }
}

void refresh_mouse_buttons(GLFWwindow* window)
{
    auto& b = hub_backend_ref();
    b.mouse_left = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    b.mouse_right = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
    b.mouse_middle = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
}

void chained_key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    auto& b = hub_backend_ref();
    const bool capture = is_valid_key_code(key) && b.capture_mask[key];

    if (!capture && b.prev_key != nullptr)
    {
        b.prev_key(window, key, scancode, action, mods);
    }

    {
        const std::lock_guard<std::mutex> lock(b.mutex);
        set_key_down(key, action);
    }

    if (b.on_key)
    {
        b.on_key(static_cast<input::key>(key), scancode, static_cast<input::key_action>(action), mods);
    }
}

void chained_mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
    auto& b = hub_backend_ref();
    if (b.prev_mouse_button != nullptr)
    {
        b.prev_mouse_button(window, button, action, mods);
    }

    const std::lock_guard<std::mutex> lock(b.mutex);
    refresh_mouse_buttons(window);
}

void chained_cursor_pos_callback(GLFWwindow* window, double x, double y)
{
    auto& b = hub_backend_ref();
    if (b.prev_cursor_pos != nullptr)
    {
        b.prev_cursor_pos(window, x, y);
    }

    const std::lock_guard<std::mutex> lock(b.mutex);
    b.mouse_dx += x - b.mouse_x;
    b.mouse_dy += y - b.mouse_y;
    b.mouse_x = x;
    b.mouse_y = y;
    refresh_mouse_buttons(window);
}

void chained_scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    auto& b = hub_backend_ref();
    if (b.prev_scroll != nullptr)
    {
        b.prev_scroll(window, xoffset, yoffset);
    }

    const std::lock_guard<std::mutex> lock(b.mutex);
    b.scroll_x += xoffset;
    b.scroll_y += yoffset;
    refresh_mouse_buttons(window);
}

}  // namespace

struct hub::impl
{
    hub_backend& state = hub_backend_ref();
};

hub& hub::instance()
{
    static hub instance;
    if (!instance.impl_)
    {
        instance.impl_ = std::make_unique<impl>();
    }
    return instance;
}

void hub::attach(GLFWwindow* window)
{
    auto& b = instance().impl_->state;
    if (window == nullptr || window == b.attached_window)
    {
        return;
    }

    b.window = window;
    b.attached_window = window;

    double x = 0.0;
    double y = 0.0;
    glfwGetCursorPos(window, &x, &y);

    {
        const std::lock_guard<std::mutex> lock(b.mutex);
        b.mouse_x = x;
        b.mouse_y = y;
        b.mouse_dx = 0.0;
        b.mouse_dy = 0.0;
        b.scroll_x = 0.0;
        b.scroll_y = 0.0;
        refresh_mouse_buttons(window);
    }

    b.prev_key = glfwSetKeyCallback(window, chained_key_callback);
    b.prev_mouse_button = glfwSetMouseButtonCallback(window, chained_mouse_button_callback);
    b.prev_cursor_pos = glfwSetCursorPosCallback(window, chained_cursor_pos_callback);
    b.prev_scroll = glfwSetScrollCallback(window, chained_scroll_callback);
}

void hub::set_key_handler(key_handler handler)
{
    instance().impl_->state.on_key = std::move(handler);
}

void hub::capture_key(key k)
{
    auto& b = instance().impl_->state;
    const int code = key_code(k);
    if (is_valid_key_code(code))
    {
        b.capture_mask[code] = true;
    }
}

void hub::release_key(key k)
{
    auto& b = instance().impl_->state;
    const int code = key_code(k);
    if (is_valid_key_code(code))
    {
        b.capture_mask[code] = false;
    }
}

void hub::clear_captured_keys()
{
    auto& b = instance().impl_->state;
    for (int i = 0; i < key_capacity; ++i)
    {
        b.capture_mask[i] = false;
    }
}

void hub::capture_wasd_keys()
{
    capture_key(key::w);
    capture_key(key::a);
    capture_key(key::s);
    capture_key(key::d);
}

void hub::capture_arrow_keys()
{
    capture_key(key::up);
    capture_key(key::down);
    capture_key(key::left);
    capture_key(key::right);
}

void hub::capture_control_keys()
{
    capture_wasd_keys();
    capture_arrow_keys();
    // Used by controller for enable/mode switches.
    capture_key(key::space);
    capture_key(key::q);
    capture_key(key::e);
    capture_key(key::f);
    capture_key(key::backspace);
}

snapshot hub::take_snapshot()
{
    snapshot out{};
    auto* impl = instance().impl_.get();
    if (impl == nullptr)
    {
        return out;
    }

    auto& b = impl->state;
    const std::lock_guard<std::mutex> lock(b.mutex);

    if (b.window != nullptr)
    {
        refresh_mouse_buttons(b.window);
    }

    out.keyboard = {};
    for (int i = 0; i < key_capacity; ++i)
    {
        out.keyboard.down[i] = b.key_down[i];
    }

    out.mouse.x = b.mouse_x;
    out.mouse.y = b.mouse_y;
    out.mouse.dx = b.mouse_dx;
    out.mouse.dy = b.mouse_dy;
    out.mouse.scroll_x = b.scroll_x;
    out.mouse.scroll_y = b.scroll_y;
    out.mouse.left = b.mouse_left;
    out.mouse.right = b.mouse_right;
    out.mouse.middle = b.mouse_middle;

    b.mouse_dx = 0.0;
    b.mouse_dy = 0.0;
    b.scroll_x = 0.0;
    b.scroll_y = 0.0;

    return out;
}

}  // namespace mujoco_interface::input
