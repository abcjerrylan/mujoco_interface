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
#include <cstdint>
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

constexpr auto k_viewer_sync_period = std::chrono::milliseconds(16);
constexpr auto k_max_pacing_lag = std::chrono::milliseconds(100);

using Seconds = std::chrono::duration<double>;

struct run_options
{
    std::string config_path;
    std::string scene_path;
    std::string topic_namespace = "mujoco_sim";
    bool topic_namespace_set = false;
    bool headless = false;
    bool show_help = false;
    std::uint64_t max_steps = 0;
    std::chrono::milliseconds metrics_period{1000};
    std::chrono::microseconds commit_timeout{core::k_default_commit_timeout};
    std::uint64_t command_hold_ticks = 5;
};

struct runtime_metrics
{
    std::chrono::steady_clock::time_point started_at{};
    std::chrono::steady_clock::time_point window_started_at{};
    double start_sim_time = 0.0;
    double window_start_sim_time = 0.0;
    std::uint64_t steps = 0;
    std::uint64_t window_start_steps = 0;

    void reset(double sim_time)
    {
        started_at = std::chrono::steady_clock::now();
        window_started_at = started_at;
        start_sim_time = sim_time;
        window_start_sim_time = sim_time;
        steps = 0;
        window_start_steps = 0;
    }
};

struct viewer_controls
{
    bool run = true;
    bool busywait = false;
    bool speed_changed = false;
    int real_time_index = 0;
};

struct viewer_control_state
{
    void store(const viewer_controls& controls)
    {
        run.store(controls.run, std::memory_order_relaxed);
        busywait.store(controls.busywait, std::memory_order_relaxed);
        real_time_index.store(controls.real_time_index, std::memory_order_relaxed);
        if (controls.speed_changed)
        {
            speed_changed.store(true, std::memory_order_release);
        }
    }

    viewer_controls load()
    {
        viewer_controls controls;
        controls.run = run.load(std::memory_order_relaxed);
        controls.busywait = busywait.load(std::memory_order_relaxed);
        controls.real_time_index = real_time_index.load(std::memory_order_relaxed);
        controls.speed_changed = speed_changed.exchange(false, std::memory_order_acq_rel);
        return controls;
    }

    std::atomic<bool> run{true};
    std::atomic<bool> busywait{false};
    std::atomic<bool> speed_changed{false};
    std::atomic<int> real_time_index{0};
};

struct runtime_context
{
    std::optional<core::simulation> simulation;
    transport::server transport;
    std::atomic<bool> exit_request{false};
    std::atomic<bool> pending_home_reset{false};
    std::unique_ptr<mjModel, void (*)(mjModel*)> viewer_model{nullptr, mj_deleteModel};
    std::unique_ptr<mjData, void (*)(mjData*)> viewer_data{nullptr, mj_deleteData};
    runtime_metrics metrics;
    viewer_control_state viewer_control;
};

runtime_context& global_context()
{
    static runtime_context ctx;
    return ctx;
}

void print_usage(const char* prog)
{
    std::printf("Usage: %s [-c config/robots/NAME.yaml] [scene.xml] [--headless] [--topic-ns NAME] "
                "[--max-steps N] [--metrics-period-ms N] [--commit-timeout-us N] "
                "[--command-hold-ticks N]\n"
                "  --metrics-period-ms N   print realtime metrics every N ms (default 1000, 0 disables)\n"
                "  --commit-timeout-us N   wait this long for each tick commit (default %lld)\n"
                "  --command-hold-ticks N  hold the last command for N missing commits, then apply zero (default 5)\n",
                prog, static_cast<long long>(core::k_default_commit_timeout.count()));
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

void maybe_print_metrics(runtime_context& ctx, const run_options& opts, bool final)
{
    if (opts.metrics_period.count() <= 0)
    {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (!final && now - ctx.metrics.window_started_at < opts.metrics_period)
    {
        return;
    }

    const double sim_time = ctx.simulation->simulation_time();
    const double window_wall = std::max(std::chrono::duration<double>(now - ctx.metrics.window_started_at).count(), 1e-9);
    const double total_wall = std::max(std::chrono::duration<double>(now - ctx.metrics.started_at).count(), 1e-9);
    const std::uint64_t window_steps = ctx.metrics.steps - ctx.metrics.window_start_steps;
    const double window_sim = sim_time - ctx.metrics.window_start_sim_time;
    const double total_sim = sim_time - ctx.metrics.start_sim_time;

    std::printf("sim metrics%s steps=%llu sim_time=%.6f window_step_rate_hz=%.1f "
                "window_real_time_rate=%.3f total_step_rate_hz=%.1f total_real_time_rate=%.3f\n",
                final ? " final" : "",
                static_cast<unsigned long long>(ctx.metrics.steps), sim_time,
                static_cast<double>(window_steps) / window_wall, window_sim / window_wall,
                static_cast<double>(ctx.metrics.steps) / total_wall, total_sim / total_wall);
    std::fflush(stdout);

    ctx.metrics.window_started_at = now;
    ctx.metrics.window_start_sim_time = sim_time;
    ctx.metrics.window_start_steps = ctx.metrics.steps;
}

bool record_step(runtime_context& ctx, const run_options& opts)
{
    ++ctx.metrics.steps;
    maybe_print_metrics(ctx, opts, false);
    if (opts.max_steps > 0 && ctx.metrics.steps >= opts.max_steps)
    {
        ctx.exit_request.store(true);
        return false;
    }
    return true;
}

void headless_loop(runtime_context& ctx, const run_options& opts)
{
    std::string error;
    auto next_step_at = std::chrono::steady_clock::now();
    while (!ctx.exit_request.load())
    {
        if (!ctx.simulation->step(ctx.transport, error))
        {
            std::fprintf(stderr, "Simulation step failed: %s\n", error.c_str());
            break;
        }
        if (!record_step(ctx, opts))
        {
            break;
        }

        const double timestep = ctx.simulation->timestep();
        if (timestep > 0.0)
        {
            next_step_at += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(timestep));
            const auto now = std::chrono::steady_clock::now();
            if (next_step_at > now)
            {
                std::this_thread::sleep_until(next_step_at);
            }
            else
            {
                next_step_at = now;
            }
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    maybe_print_metrics(ctx, opts, true);
}

bool queue_viewer_load(mj::Simulate& sim, runtime_context& ctx, const std::string& scene_path)
{
    ctx.viewer_model.reset(ctx.simulation->copy_model());
    if (ctx.viewer_model == nullptr)
    {
        std::fprintf(stderr, "Simulation viewer load failed: failed to copy mjModel\n");
        return false;
    }
    ctx.viewer_data.reset(ctx.simulation->copy_data_for(ctx.viewer_model.get()));
    if (ctx.viewer_data == nullptr)
    {
        std::fprintf(stderr, "Simulation viewer load failed: failed to copy mjData\n");
        ctx.viewer_model.reset();
        return false;
    }
    {
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);
        sim.mnew_ = ctx.viewer_model.get();
        sim.dnew_ = ctx.viewer_data.get();
        std::snprintf(sim.filename, sizeof(sim.filename), "%s", scene_path.c_str());
        sim.loadrequest = 2;
    }
    return true;
}

bool synchronize_viewer(mj::Simulate& sim, runtime_context& ctx)
{
    if (ctx.viewer_model == nullptr || ctx.viewer_data == nullptr || sim.loadrequest != 0)
    {
        return true;
    }

    std::string error;
    if (!ctx.simulation->copy_data_to(ctx.viewer_model.get(), ctx.viewer_data.get(), error))
    {
        std::fprintf(stderr, "Simulation viewer sync failed: %s\n", error.c_str());
        return false;
    }

    const std::unique_lock<std::recursive_mutex> lock(sim.mtx);
    sim.Sync();
    sim.AddToHistory();
    return true;
}

viewer_controls read_viewer_controls(mj::Simulate& sim)
{
    const std::unique_lock<std::recursive_mutex> lock(sim.mtx);
    viewer_controls controls;
    controls.run = sim.run;
    controls.busywait = sim.busywait;
    controls.speed_changed = sim.speed_changed;
    constexpr int k_num_real_time_options =
        static_cast<int>(sizeof(mj::Simulate::percentRealTime) / sizeof(mj::Simulate::percentRealTime[0]));
    controls.real_time_index = std::clamp(sim.real_time_index, 0, k_num_real_time_options - 1);
    sim.speed_changed = false;
    return controls;
}

bool simulation_step(runtime_context& ctx, const run_options& opts, std::string& error)
{
    if (!ctx.simulation->step(ctx.transport, error))
    {
        return false;
    }
    record_step(ctx, opts);
    return true;
}

void physics_loop(mj::Simulate& sim, runtime_context& ctx, const run_options& opts)
{
    auto next_step_at = mj::Simulate::Clock::now();
    bool pacing_initialized = false;

    while (!ctx.exit_request.load())
    {
        const auto loop_started_at = mj::Simulate::Clock::now();
        viewer_controls controls = ctx.viewer_control.load();

        if (ctx.pending_home_reset.exchange(false))
        {
            ctx.simulation->request_reset();
        }

        if (controls.run)
        {
            std::string error;
            if (!simulation_step(ctx, opts, error))
            {
                std::fprintf(stderr, "Simulation step failed: %s\n", error.c_str());
                ctx.exit_request.store(true);
                break;
            }

            const double timestep = ctx.simulation->timestep();
            const double slowdown = 100.0 / mj::Simulate::percentRealTime[controls.real_time_index];
            const auto step_period = std::chrono::duration_cast<mj::Simulate::Clock::duration>(
                Seconds(timestep > 0.0 ? timestep * slowdown : 0.001));

            if (!pacing_initialized || controls.speed_changed)
            {
                next_step_at = loop_started_at;
                pacing_initialized = true;
                controls.speed_changed = false;
            }
            next_step_at += step_period;

            publish_control_input(ctx.transport);

            const auto now = mj::Simulate::Clock::now();
            if (next_step_at > now)
            {
                if (controls.busywait)
                {
                    while (!ctx.exit_request.load() && mj::Simulate::Clock::now() < next_step_at)
                    {
                        std::this_thread::yield();
                    }
                }
                else
                {
                    std::this_thread::sleep_until(next_step_at);
                }
            }
            else if (now - next_step_at > k_max_pacing_lag)
            {
                next_step_at = now;
            }
        }
        else
        {
            pacing_initialized = false;
            publish_control_input(ctx.transport);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    maybe_print_metrics(ctx, opts, true);
}

void viewer_sync_loop(mj::Simulate& sim, runtime_context& ctx)
{
    while (!ctx.exit_request.load() && !sim.exitrequest.load())
    {
        if (!synchronize_viewer(sim, ctx))
        {
            ctx.exit_request.store(true);
            sim.exitrequest.store(true);
            break;
        }
        ctx.viewer_control.store(read_viewer_controls(sim));
        std::this_thread::sleep_for(k_viewer_sync_period);
    }
}

}  // namespace

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
        else if (std::strcmp(argv[i], "--max-steps") == 0 && i + 1 < argc)
        {
            opts.max_steps = std::strtoull(argv[++i], nullptr, 10);
        }
        else if (std::strcmp(argv[i], "--metrics-period-ms") == 0 && i + 1 < argc)
        {
            opts.metrics_period = std::chrono::milliseconds(std::strtoll(argv[++i], nullptr, 10));
        }
        else if (std::strcmp(argv[i], "--commit-timeout-us") == 0 && i + 1 < argc)
        {
            opts.commit_timeout = std::chrono::microseconds(std::strtoll(argv[++i], nullptr, 10));
        }
        else if (std::strcmp(argv[i], "--command-hold-ticks") == 0 && i + 1 < argc)
        {
            opts.command_hold_ticks = std::strtoull(argv[++i], nullptr, 10);
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
    sim_cfg.commit_timeout = opts.commit_timeout;
    sim_cfg.command_hold_ticks = opts.command_hold_ticks;

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
    ctx.metrics.reset(ctx.simulation->simulation_time());

    if (opts.headless)
    {
        headless_loop(ctx, opts);
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

    if (!queue_viewer_load(*sim, ctx, scene_path))
    {
        ctx.simulation->shutdown(ctx.transport);
        return 1;
    }
    ctx.viewer_control.store(read_viewer_controls(*sim));

    std::thread phys_thread([&sim, &ctx, &opts]() { physics_loop(*sim, ctx, opts); });
    std::thread viewer_sync_thread([&sim, &ctx]() { viewer_sync_loop(*sim, ctx); });

    sim->RenderLoop();
    sim->exitrequest.store(true);
    ctx.exit_request.store(true);
    phys_thread.join();
    viewer_sync_thread.join();

    ctx.simulation->shutdown(ctx.transport);
    return 0;
}

}  // namespace mujoco_interface::sim
