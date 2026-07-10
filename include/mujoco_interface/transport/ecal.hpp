#pragma once

#include "mujoco_interface/protocol/messages.hpp"

#include <ecal/ecal.h>
#include <ecal/pubsub/publisher.h>
#include <ecal/pubsub/subscriber.h>

#include <functional>
#include <memory>
#include <mutex>
#include <string>

namespace mujoco_interface::transport
{

struct callbacks
{
    std::function<void(const protocol::register_message&)> on_register;
    std::function<void(const protocol::command_envelope&)> on_commit;
};

class server
{
public:
    server();
    ~server();

    bool start(const std::string& topic_namespace, const callbacks& callbacks, std::string& error);
    void stop();
    void publish_tick(const protocol::tick_message& tick);
    void publish_state(const protocol::state_envelope& state);
    void publish_register_ack(const protocol::register_ack_message& ack);
    void publish_input(const protocol::input_message& input);
    void poll();

private:
    std::mutex mutex_;
    std::string topic_namespace_;
    callbacks callbacks_{};
    bool running_ = false;
    bool ecal_initialized_ = false;
    std::unique_ptr<eCAL::CPublisher> tick_pub_;
    std::unique_ptr<eCAL::CPublisher> state_pub_;
    std::unique_ptr<eCAL::CPublisher> register_ack_pub_;
    std::unique_ptr<eCAL::CPublisher> input_pub_;
    std::unique_ptr<eCAL::CSubscriber> register_sub_;
    std::unique_ptr<eCAL::CSubscriber> commit_sub_;
};

class client
{
public:
    client();
    ~client();

    bool start(const std::string& topic_namespace, const std::string& unit_name, std::string& error);
    void stop();
    void poll();
    bool register_client(const protocol::register_message& request, std::string& error);
    bool send_commit(const protocol::command_envelope& commit);

    void set_register_ack_handler(std::function<void(const protocol::register_ack_message&)> handler);
    void set_tick_handler(std::function<void(const protocol::tick_message&)> handler);
    void set_state_handler(std::function<void(const protocol::state_envelope&)> handler);
    void set_input_handler(std::function<void(const protocol::input_message&)> handler);

private:
    std::mutex mutex_;
    bool running_ = false;
    bool ecal_initialized_ = false;
    std::unique_ptr<eCAL::CPublisher> register_pub_;
    std::unique_ptr<eCAL::CPublisher> commit_pub_;
    std::unique_ptr<eCAL::CSubscriber> register_ack_sub_;
    std::unique_ptr<eCAL::CSubscriber> tick_sub_;
    std::unique_ptr<eCAL::CSubscriber> state_sub_;
    std::unique_ptr<eCAL::CSubscriber> input_sub_;
    std::function<void(const protocol::register_ack_message&)> on_register_ack_;
    std::function<void(const protocol::tick_message&)> on_tick_;
    std::function<void(const protocol::state_envelope&)> on_state_;
    std::function<void(const protocol::input_message&)> on_input_;
};

}  // namespace mujoco_interface::transport
