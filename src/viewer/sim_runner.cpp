#include "mujoco_interface/sim/runner.hpp"
#include "mujoco_interface/core/simulation.hpp"
#include "mujoco_interface/input_hub.hpp"
#include "mujoco_interface/robot_config.hpp"

#include <yaml-cpp/yaml.h>

#define private public
#include "glfw_adapter.h"
#undef private

#include <mujoco/mujoco.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "array_safety.h"
#include "simulate.h"

namespace mujoco_interface::sim
{

namespace
{

namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;

constexpr double k_sync_misalign = 0.1;
constexpr double k_sim_refresh_fraction = 0.7;

using Seconds = std::chrono::duration<double>;

struct runtime_context
{
    std::optional<core::simulation> simulation;
    transport::server transport;
    std::atomic<bool> exit_request{false};
    std::atomic<bool> pending_home_reset{false};
};

runtime_context& global_context()
{
    static runtime_context ctx;
    return ctx;
}

void print_usage(const char* prog)
{
    std::printf("Usage: %s [-c config/robots/NAME.yaml] [scene.xml] [--headless] [--topic-ns NAME]\n", prog);
}

std::string resolve_scene_fallback()
{
    const std::filesystem::path candidates[] = {
        "scene.xml",
        "config/scene.xml",
        "../scene.xml",
    };

    for (const auto& candidate : candidates)
    {
        if (std::filesystem::exists(candidate))
        {
            return std::filesystem::absolute(candidate).string();
        }
    }

    return "scene.xml";
}

protocol::input_message snapshot_to_input(const input::snapshot& snap)
{
    const auto& kb = snap.keyboard;
    protocol::input_message msg{};
    msg.w = kb.is_down(static_cast<int>(input::key::w));
    msg.s = kb.is_down(static_cast<int>(input::key::s));
    msg.a = kb.is_down(static_cast<int>(input::key::a));
    msg.d = kb.is_down(static_cast<int>(input::key::d));
    msg.q = kb.is_down(static_cast<int>(input::key::q));
    msg.e = kb.is_down(static_cast<int>(input::key::e));
    msg.f = kb.is_down(static_cast<int>(input::key::f));
    msg.r = kb.is_down(static_cast<int>(input::key::r));
    msg.space = kb.is_down(static_cast<int>(input::key::space));
    return msg;
}

void publish_control_input(transport::server& transport)
{
    transport.publish_input(snapshot_to_input(input::hub::instance().take_snapshot()));
}

void headless_loop(runtime_context& ctx)
{
    std::string error;
    while (!ctx.exit_request.load())
    {
        if (!ctx.simulation->step(ctx.transport, error))
        {
            std::fprintf(stderr, "Simulation step failed: %s\n", error.c_str());
            break;
        }

        const double timestep = ctx.simulation->models().robot().sim_timestep();
        if (timestep > 0.0)
        {
            std::this_thread::sleep_for(std::chrono::duration<double>(timestep));
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

void physics_loop(mj::Simulate& sim, runtime_context& ctx, const std::string& scene_path)
{
    sim.Load(ctx.simulation->models().model(), ctx.simulation->models().data(), scene_path.c_str());

    std::chrono::time_point<mj::Simulate::Clock> sync_cpu;
    mjtNum sync_sim = 0;

    while (!sim.exitrequest.load())
    {
        if (sim.run && sim.busywait)
        {
            std::this_thread::yield();
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        {
            const std::unique_lock<std::recursive_mutex> lock(sim.mtx);
            auto& models = ctx.simulation->models();
            if (!models.model() || !models.data())
            {
                continue;
            }

            if (ctx.pending_home_reset.exchange(false))
            {
                ctx.simulation->request_reset();
            }

            if (sim.run)
            {
                bool stepped = false;
                const auto start_cpu = mj::Simulate::Clock::now();
                const auto elapsed_cpu = start_cpu - sync_cpu;
                const double elapsed_sim = models.data()->time - sync_sim;

                std::string error;
                const double slowdown = 100 / sim.percentRealTime[sim.real_time_index];
                const bool misaligned =
                    mju_abs(Seconds(elapsed_cpu).count() / slowdown - elapsed_sim) > k_sync_misalign;

                if (elapsed_sim < 0 || elapsed_cpu.count() < 0 || sync_cpu.time_since_epoch().count() == 0 ||
                    misaligned || sim.speed_changed)
                {
                    sync_cpu = start_cpu;
                    sync_sim = models.data()->time;
                    sim.speed_changed = false;
                    if (ctx.simulation->step(ctx.transport, error))
                    {
                        stepped = true;
                    }
                    else
                    {
                        std::fprintf(stderr, "Simulation step failed: %s\n", error.c_str());
                    }
                }
                else
                {
                    const double refresh_time = k_sim_refresh_fraction / sim.refresh_rate;
                    while (Seconds((models.data()->time - sync_sim) * slowdown) < mj::Simulate::Clock::now() - sync_cpu &&
                           mj::Simulate::Clock::now() - start_cpu < Seconds(refresh_time))
                    {
                        if (ctx.simulation->step(ctx.transport, error))
                        {
                            stepped = true;
                        }
                        else
                        {
                            std::fprintf(stderr, "Simulation step failed: %s\n", error.c_str());
                            break;
                        }
                    }
                }

                if (stepped)
                {
                    sim.Sync();
                    sim.AddToHistory();
                }
            }
            else
            {
                mj_forward(models.model(), models.data());
                sim.Sync();
                sim.speed_changed = true;
            }
        }

        publish_control_input(ctx.transport);
    }
}

}  // namespace

struct run_options
{
    std::string config_path;
    std::string scene_path;
    std::string topic_namespace = "mujoco_sim";
    bool topic_namespace_set = false;
    bool headless = false;
    bool show_help = false;
};

run_options parse_args(int argc, char** argv)
{
    run_options opts;
    for (int i = 1; i < argc; ++i)
    {
        if (std::strcmp(argv[i], "-h") == 0 || std::strcmp(argv[i], "--help") == 0)
        {
            opts.show_help = true;
            return opts;
        }
        if ((std::strcmp(argv[i], "-c") == 0 || std::strcmp(argv[i], "--config") == 0) && i + 1 < argc)
        {
            opts.config_path = argv[++i];
        }
        else if (std::strcmp(argv[i], "--headless") == 0)
        {
            opts.headless = true;
        }
        else if (std::strcmp(argv[i], "--topic-ns") == 0 && i + 1 < argc)
        {
            opts.topic_namespace = argv[++i];
            opts.topic_namespace_set = true;
        }
        else if (opts.scene_path.empty())
        {
            opts.scene_path = argv[i];
        }
    }
    return opts;
}

int run(int argc, char** argv)
{
    auto& ctx = global_context();
    run_options opts = parse_args(argc, argv);
    if (opts.show_help)
    {
        print_usage(argv[0]);
        return 0;
    }

    if (opts.config_path.empty())
    {
        std::fprintf(stderr, "Robot config is required: -c config/robots/NAME.yaml\n");
        return 1;
    }

    std::printf("MuJoCo version %s\n", mj_versionString());
    if (mjVERSION_HEADER != mj_version())
    {
        mju_error("Headers and library have different versions");
    }

    std::string scene_path = opts.scene_path;
    robot::config robot_cfg{};
    std::string config_error;
    if (!robot::load_config(opts.config_path, robot_cfg, config_error))
    {
        std::fprintf(stderr, "Config error: %s\n", config_error.c_str());
        return 1;
    }
    if (scene_path.empty())
    {
        scene_path = robot::resolve_path(opts.config_path, robot_cfg.scene);
    }
    if (scene_path.empty())
    {
        scene_path = resolve_scene_fallback();
    }

    if (!opts.topic_namespace_set && !opts.config_path.empty())
    {
        try
        {
            const YAML::Node root = YAML::LoadFile(opts.config_path);
            if (root["ipc_prefix"])
            {
                opts.topic_namespace = root["ipc_prefix"].as<std::string>();
            }
        }
        catch (...)
        {
        }
    }

    core::simulation_config sim_cfg{};
    sim_cfg.topic_namespace = opts.topic_namespace;

    ctx.simulation.emplace(sim_cfg);
    std::string error;
    if (!ctx.simulation->init(opts.config_path, scene_path, ctx.transport, error))
    {
        std::fprintf(stderr, "Simulation init failed: %s\n", error.c_str());
        return 1;
    }

    std::printf("Loading scene: %s\n", scene_path.c_str());
    std::printf("Robot config: %s\n", opts.config_path.c_str());
    std::printf("Topic namespace: %s\n", opts.topic_namespace.c_str());

    if (opts.headless)
    {
        headless_loop(ctx);
        ctx.simulation->shutdown(ctx.transport);
        return 0;
    }

    mjvCamera cam;
    mjv_defaultCamera(&cam);

    mjvOption opt;
    mjv_defaultOption(&opt);

    mjvPerturb pert;
    mjv_defaultPerturb(&pert);

    auto sim = std::make_unique<mj::Simulate>(
        std::make_unique<mj::GlfwAdapter>(),
        &cam, &opt, &pert, /* is_passive = */ true);

    auto* glfw_adapter = static_cast<mj::GlfwAdapter*>(sim->platform_ui.get());
    input::hub::instance().attach(glfw_adapter->window_);
    input::hub::instance().capture_control_keys();
    input::hub::instance().set_key_handler(
        [](input::key k, int, input::key_action action, int)
        {
            if (action == input::key_action::press && k == input::key::backspace)
            {
                global_context().pending_home_reset.store(true);
            }
        });

    std::thread phys_thread(
        [&sim, &ctx, scene_path]()
        {
            physics_loop(*sim, ctx, scene_path);
        });

    sim->RenderLoop();
    sim->exitrequest.store(true);
    ctx.exit_request.store(true);
    phys_thread.join();

    ctx.simulation->shutdown(ctx.transport);
    return 0;
}

}  // namespace mujoco_interface::sim
