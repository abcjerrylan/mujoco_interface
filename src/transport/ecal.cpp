#include "mujoco_interface/transport/ecal.hpp"

#include <ecal/core.h>
#include <ecal/config/configuration.h>
#include <ecal/pubsub/types.h>
#include <ecal/types.h>

#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <limits.h>
#include <string>
#include <unistd.h>

#ifndef MUJOCO_INTERFACE_ECAL_DATA
#define MUJOCO_INTERFACE_ECAL_DATA ""
#endif

namespace mujoco_interface::transport
{

namespace detail
{

inline eCAL::SDataTypeInformation make_type_info()
{
    eCAL::SDataTypeInformation info;
    info.name = "mujoco_interface";
    info.encoding = "binary";
    info.descriptor.clear();
    return info;
}

template <typename T>
inline bool validate_blob(const eCAL::SReceiveCallbackData& data, T& out)
{
    if (data.buffer_size < sizeof(T))
    {
        return false;
    }
    std::memcpy(&out, data.buffer, sizeof(T));
    return true;
}

template <typename T>
inline void publish_blob(eCAL::CPublisher& publisher, const T& message)
{
    publisher.Send(&message, sizeof(T));
}

inline std::string installed_ecal_data_dir()
{
    char exe_path[PATH_MAX] = {};
    const ssize_t len = readlink("/proc/self/exe", exe_path, sizeof(exe_path) - 1);
    if (len <= 0)
    {
        return {};
    }
    exe_path[len] = '\0';

    const std::filesystem::path prefix =
        std::filesystem::path(exe_path).parent_path().parent_path();
    const std::filesystem::path ecal_data = prefix / "share" / "mujoco_interface" / "ecal";
    if (std::filesystem::exists(ecal_data / "ecal.yaml"))
    {
        return ecal_data.string();
    }
    return {};
}

inline void prepare_ecal_runtime()
{
    if (const char* current = std::getenv("ECAL_DATA"); current != nullptr && current[0] != '\0')
    {
        return;
    }

    const std::string installed = installed_ecal_data_dir();
    if (!installed.empty())
    {
        setenv("ECAL_DATA", installed.c_str(), 0);
        return;
    }

#if defined(MUJOCO_INTERFACE_ECAL_DATA)
    if (MUJOCO_INTERFACE_ECAL_DATA[0] != '\0')
    {
        setenv("ECAL_DATA", MUJOCO_INTERFACE_ECAL_DATA, 0);
    }
#endif
}

inline unsigned ecal_init_components()
{
    // Tick sync is handled by mujoco_interface core; skip eCAL time plugins.
    return eCAL::Init::Default & ~eCAL::Init::TimeSync;
}

inline bool initialize_ecal(const std::string& unit_name)
{
    prepare_ecal_runtime();

    eCAL::Configuration config;
    if (const char* ecal_data = std::getenv("ECAL_DATA"); ecal_data != nullptr && ecal_data[0] != '\0')
    {
        const std::filesystem::path config_file = std::filesystem::path(ecal_data) / "ecal.yaml";
        if (std::filesystem::exists(config_file))
        {
            config.InitFromFile(config_file.string());
        }
        else
        {
            config.InitFromConfig();
        }
    }
#if defined(MUJOCO_INTERFACE_ECAL_DATA)
    else if (MUJOCO_INTERFACE_ECAL_DATA[0] != '\0')
    {
        config.InitFromFile(std::string(MUJOCO_INTERFACE_ECAL_DATA) + "/ecal.yaml");
    }
#endif
    else
    {
        config.InitFromConfig();
    }
    return eCAL::Initialize(config, unit_name, ecal_init_components());
}

}  // namespace detail

namespace
{

std::string topic_name(const std::string& ns, const char* suffix)
{
    return ns + "/" + suffix;
}

}  // namespace

server::server() = default;

server::~server()
{
    stop();
}

bool server::start(const std::string& topic_namespace, const callbacks& transport_callbacks, std::string& error)
{
    const std::lock_guard<std::mutex> lock(mutex_);
    if (running_)
    {
        return true;
    }

    if (!eCAL::IsInitialized())
    {
        if (!detail::initialize_ecal("mujoco_interface_sim"))
        {
            error = "eCAL::Initialize failed";
            return false;
        }
        ecal_initialized_ = true;
    }

    topic_namespace_ = topic_namespace;
    callbacks_ = transport_callbacks;
    const eCAL::SDataTypeInformation type_info = detail::make_type_info();

    tick_pub_ = std::make_unique<eCAL::CPublisher>(topic_name(topic_namespace_, "tick"), type_info);
    state_pub_ = std::make_unique<eCAL::CPublisher>(topic_name(topic_namespace_, "state"), type_info);
    register_ack_pub_ =
        std::make_unique<eCAL::CPublisher>(topic_name(topic_namespace_, "register_ack"), type_info);
    input_pub_ = std::make_unique<eCAL::CPublisher>(topic_name(topic_namespace_, "input"), type_info);
    register_sub_ = std::make_unique<eCAL::CSubscriber>(topic_name(topic_namespace_, "register"), type_info);
    commit_sub_ = std::make_unique<eCAL::CSubscriber>(topic_name(topic_namespace_, "commit"), type_info);

    register_sub_->SetReceiveCallback(
        [this](const eCAL::STopicId&, const eCAL::SDataTypeInformation&, const eCAL::SReceiveCallbackData& data)
        {
            protocol::register_message request{};
            if (!detail::validate_blob(data, request))
            {
                return;
            }
            callbacks cb_copy;
            {
                const std::lock_guard<std::mutex> inner_lock(mutex_);
                cb_copy = callbacks_;
            }
            if (cb_copy.on_register)
            {
                cb_copy.on_register(request);
            }
        });

    commit_sub_->SetReceiveCallback(
        [this](const eCAL::STopicId&, const eCAL::SDataTypeInformation&, const eCAL::SReceiveCallbackData& data)
        {
            protocol::command_envelope commit{};
            if (!detail::validate_blob(data, commit))
            {
                return;
            }
            callbacks cb_copy;
            {
                const std::lock_guard<std::mutex> inner_lock(mutex_);
                cb_copy = callbacks_;
            }
            if (cb_copy.on_commit)
            {
                cb_copy.on_commit(commit);
            }
        });

    running_ = true;
    return true;
}

void server::stop()
{
    const std::lock_guard<std::mutex> lock(mutex_);
    register_sub_.reset();
    commit_sub_.reset();
    tick_pub_.reset();
    state_pub_.reset();
    register_ack_pub_.reset();
    input_pub_.reset();
    running_ = false;
    if (ecal_initialized_ && eCAL::IsInitialized())
    {
        eCAL::Finalize();
        ecal_initialized_ = false;
    }
}

void server::publish_tick(const protocol::tick_message& tick)
{
    const std::lock_guard<std::mutex> lock(mutex_);
    if (tick_pub_)
    {
        detail::publish_blob(*tick_pub_, tick);
    }
}

void server::publish_state(const protocol::state_envelope& state)
{
    const std::lock_guard<std::mutex> lock(mutex_);
    if (state_pub_)
    {
        detail::publish_blob(*state_pub_, state);
    }
}

void server::publish_register_ack(const protocol::register_ack_message& ack)
{
    const std::lock_guard<std::mutex> lock(mutex_);
    if (register_ack_pub_)
    {
        detail::publish_blob(*register_ack_pub_, ack);
    }
}

void server::publish_input(const protocol::input_message& input)
{
    const std::lock_guard<std::mutex> lock(mutex_);
    if (input_pub_)
    {
        detail::publish_blob(*input_pub_, input);
    }
}

void server::poll()
{
    (void)eCAL::Ok();
}

client::client() = default;

client::~client()
{
    stop();
}

bool client::start(const std::string& topic_namespace, const std::string& unit_name, std::string& error)
{
    const std::lock_guard<std::mutex> lock(mutex_);
    if (running_)
    {
        return true;
    }

    if (!eCAL::IsInitialized())
    {
        if (!detail::initialize_ecal(unit_name))
        {
            error = "eCAL::Initialize failed";
            return false;
        }
        ecal_initialized_ = true;
    }

    const eCAL::SDataTypeInformation type_info = detail::make_type_info();
    register_pub_ = std::make_unique<eCAL::CPublisher>(topic_name(topic_namespace, "register"), type_info);
    commit_pub_ = std::make_unique<eCAL::CPublisher>(topic_name(topic_namespace, "commit"), type_info);
    register_ack_sub_ =
        std::make_unique<eCAL::CSubscriber>(topic_name(topic_namespace, "register_ack"), type_info);
    tick_sub_ = std::make_unique<eCAL::CSubscriber>(topic_name(topic_namespace, "tick"), type_info);
    state_sub_ = std::make_unique<eCAL::CSubscriber>(topic_name(topic_namespace, "state"), type_info);
    input_sub_ = std::make_unique<eCAL::CSubscriber>(topic_name(topic_namespace, "input"), type_info);

    register_ack_sub_->SetReceiveCallback(
        [this](const eCAL::STopicId&, const eCAL::SDataTypeInformation&, const eCAL::SReceiveCallbackData& data)
        {
            protocol::register_ack_message ack{};
            if (!detail::validate_blob(data, ack))
            {
                return;
            }
            std::function<void(const protocol::register_ack_message&)> handler;
            {
                const std::lock_guard<std::mutex> inner_lock(mutex_);
                handler = on_register_ack_;
            }
            if (handler)
            {
                handler(ack);
            }
        });

    tick_sub_->SetReceiveCallback(
        [this](const eCAL::STopicId&, const eCAL::SDataTypeInformation&, const eCAL::SReceiveCallbackData& data)
        {
            protocol::tick_message tick{};
            if (!detail::validate_blob(data, tick))
            {
                return;
            }
            std::function<void(const protocol::tick_message&)> handler;
            {
                const std::lock_guard<std::mutex> inner_lock(mutex_);
                handler = on_tick_;
            }
            if (handler)
            {
                handler(tick);
            }
        });

    state_sub_->SetReceiveCallback(
        [this](const eCAL::STopicId&, const eCAL::SDataTypeInformation&, const eCAL::SReceiveCallbackData& data)
        {
            protocol::state_envelope state{};
            if (!detail::validate_blob(data, state))
            {
                return;
            }
            std::function<void(const protocol::state_envelope&)> handler;
            {
                const std::lock_guard<std::mutex> inner_lock(mutex_);
                handler = on_state_;
            }
            if (handler)
            {
                handler(state);
            }
        });

    input_sub_->SetReceiveCallback(
        [this](const eCAL::STopicId&, const eCAL::SDataTypeInformation&, const eCAL::SReceiveCallbackData& data)
        {
            protocol::input_message input{};
            if (!detail::validate_blob(data, input))
            {
                return;
            }
            std::function<void(const protocol::input_message&)> handler;
            {
                const std::lock_guard<std::mutex> inner_lock(mutex_);
                handler = on_input_;
            }
            if (handler)
            {
                handler(input);
            }
        });

    running_ = true;
    return true;
}

void client::stop()
{
    const std::lock_guard<std::mutex> lock(mutex_);
    register_pub_.reset();
    commit_pub_.reset();
    register_ack_sub_.reset();
    tick_sub_.reset();
    state_sub_.reset();
    input_sub_.reset();
    running_ = false;
    if (ecal_initialized_ && eCAL::IsInitialized())
    {
        eCAL::Finalize();
        ecal_initialized_ = false;
    }
}

void client::poll()
{
    (void)eCAL::Ok();
}

bool client::register_client(const protocol::register_message& request, std::string& error)
{
    const std::lock_guard<std::mutex> lock(mutex_);
    if (!register_pub_)
    {
        error = "register publisher not ready";
        return false;
    }
    detail::publish_blob(*register_pub_, request);
    return true;
}

bool client::send_commit(const protocol::command_envelope& commit)
{
    const std::lock_guard<std::mutex> lock(mutex_);
    if (!commit_pub_)
    {
        return false;
    }
    detail::publish_blob(*commit_pub_, commit);
    return true;
}

void client::set_register_ack_handler(std::function<void(const protocol::register_ack_message&)> handler)
{
    const std::lock_guard<std::mutex> lock(mutex_);
    on_register_ack_ = std::move(handler);
}

void client::set_tick_handler(std::function<void(const protocol::tick_message&)> handler)
{
    const std::lock_guard<std::mutex> lock(mutex_);
    on_tick_ = std::move(handler);
}

void client::set_state_handler(std::function<void(const protocol::state_envelope&)> handler)
{
    const std::lock_guard<std::mutex> lock(mutex_);
    on_state_ = std::move(handler);
}

void client::set_input_handler(std::function<void(const protocol::input_message&)> handler)
{
    const std::lock_guard<std::mutex> lock(mutex_);
    on_input_ = std::move(handler);
}

}  // namespace mujoco_interface::transport
