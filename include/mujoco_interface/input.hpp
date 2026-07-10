#pragma once

#include <cstdint>

namespace mujoco_interface::input
{

// GLFW key code range (see GLFW/glfw3.h). Index keys by their GLFW integer code.
inline constexpr int key_capacity = 512;
inline constexpr int last_key = 348;

enum class key_action : int
{
    release = 0,
    press = 1,
    repeat = 2,
};

enum class key_mod : int
{
    shift = 0x0001,
    control = 0x0002,
    alt = 0x0004,
    super = 0x0008,
    caps_lock = 0x0010,
    num_lock = 0x0020,
};

enum class key : int
{
    unknown = -1,

    space = 32,
    apostrophe = 39,
    comma = 44,
    minus = 45,
    period = 46,
    slash = 47,
    num_0 = 48,
    num_1 = 49,
    num_2 = 50,
    num_3 = 51,
    num_4 = 52,
    num_5 = 53,
    num_6 = 54,
    num_7 = 55,
    num_8 = 56,
    num_9 = 57,
    semicolon = 59,
    equal = 61,
    a = 65,
    b = 66,
    c = 67,
    d = 68,
    e = 69,
    f = 70,
    g = 71,
    h = 72,
    i = 73,
    j = 74,
    k = 75,
    l = 76,
    m = 77,
    n = 78,
    o = 79,
    p = 80,
    q = 81,
    r = 82,
    s = 83,
    t = 84,
    u = 85,
    v = 86,
    w = 87,
    x = 88,
    y = 89,
    z = 90,
    left_bracket = 91,
    backslash = 92,
    right_bracket = 93,
    grave_accent = 96,
    world_1 = 161,
    world_2 = 162,

    escape = 256,
    enter = 257,
    tab = 258,
    backspace = 259,
    insert = 260,
    del = 261,
    right = 262,
    left = 263,
    down = 264,
    up = 265,
    page_up = 266,
    page_down = 267,
    home = 268,
    end = 269,
    caps_lock = 280,
    scroll_lock = 281,
    num_lock = 282,
    print_screen = 283,
    pause = 284,
    f1 = 290,
    f2 = 291,
    f3 = 292,
    f4 = 293,
    f5 = 294,
    f6 = 295,
    f7 = 296,
    f8 = 297,
    f9 = 298,
    f10 = 299,
    f11 = 300,
    f12 = 301,
    f13 = 302,
    f14 = 303,
    f15 = 304,
    f16 = 305,
    f17 = 306,
    f18 = 307,
    f19 = 308,
    f20 = 309,
    f21 = 310,
    f22 = 311,
    f23 = 312,
    f24 = 313,
    f25 = 314,
    kp_0 = 320,
    kp_1 = 321,
    kp_2 = 322,
    kp_3 = 323,
    kp_4 = 324,
    kp_5 = 325,
    kp_6 = 326,
    kp_7 = 327,
    kp_8 = 328,
    kp_9 = 329,
    kp_decimal = 330,
    kp_divide = 331,
    kp_multiply = 332,
    kp_subtract = 333,
    kp_add = 334,
    kp_enter = 335,
    kp_equal = 336,
    left_shift = 340,
    left_control = 341,
    left_alt = 342,
    left_super = 343,
    right_shift = 344,
    right_control = 345,
    right_alt = 346,
    right_super = 347,
    menu = 348,
};

enum class mouse_button : int
{
    left = 0,
    right = 1,
    middle = 2,
};

inline constexpr int key_code(key k)
{
    return static_cast<int>(k);
}

inline constexpr bool is_valid_key_code(int k)
{
    return k >= 0 && k < key_capacity;
}

struct key_state
{
    bool down[key_capacity]{};

    [[nodiscard]] bool is_down(key k) const
    {
        return is_down(key_code(k));
    }

    [[nodiscard]] bool is_down(int glfw_key_code) const
    {
        if (!is_valid_key_code(glfw_key_code))
        {
            return false;
        }
        return down[glfw_key_code];
    }
};

struct mouse_state
{
    double x = 0.0;
    double y = 0.0;
    double dx = 0.0;
    double dy = 0.0;
    double scroll_x = 0.0;
    double scroll_y = 0.0;
    bool left = false;
    bool right = false;
    bool middle = false;

    [[nodiscard]] bool is_down(mouse_button b) const
    {
        switch (b)
        {
            case mouse_button::left:
                return left;
            case mouse_button::right:
                return right;
            case mouse_button::middle:
                return middle;
        }
        return false;
    }
};

struct snapshot
{
    key_state keyboard{};
    mouse_state mouse{};
};

}  // namespace mujoco_interface::input
