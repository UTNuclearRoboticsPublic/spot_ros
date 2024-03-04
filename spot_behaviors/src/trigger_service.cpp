#include "spot_behaviors/trigger_service.hpp"

namespace spot_behaviors {

TriggerService::TriggerService(const std::string& name, const BT::NodeConfiguration& config):
    BT::StatefulActionNode(name, config),
    node_(std::make_shared<rclcpp::Node>(name+"BT"+std::to_string(node_count_++)))
{}

BT::PortsList TriggerService::providedPorts() {
    return {
        BT::InputPort<float>("service_name")
    };
}

BT::NodeStatus TriggerService::onStart() {
    BT::Optional<std::string> service_name = getInput<std::string>("service_name");
    trigger_client_ = node_->create_client<std_srvs::srv::Trigger>(service_name.value());

    // Make sure the server is available to us
    if (!trigger_client_->wait_for_service(std::chrono::seconds(2))){
        RCLCPP_ERROR(node_->get_logger(), "Server \"%s\" not found, aborting", trigger_client_->get_service_name());
        return BT::NodeStatus::FAILURE;
    }

    // Make sure there aren't any existing requests - should not be possible
    if (service_future_.has_value()){
        RCLCPP_ERROR(node_->get_logger(), "Dock service called with an existing dock request. This should never happen");
        return BT::NodeStatus::FAILURE;
    }

    // Create and send the request
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    RCLCPP_INFO(node_->get_logger(), "Calling trigger request on \"%s\"", trigger_client_->get_service_name());
    
    service_future_ = trigger_client_->async_send_request(request);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TriggerService::onRunning() {
    // If there is no existing request, return failure - this should never happen
    if (!service_future_.has_value()) return BT::NodeStatus::FAILURE;

    // Check the status of the future. If it's done, return its success value
    if (service_future_->wait_for(std::chrono::seconds(0)) == std::future_status::ready){
        std_srvs::srv::Trigger::Response::SharedPtr result = service_future_->get();
        service_future_ = std::nullopt;
        return result->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    // Otherwise return running
    return BT::NodeStatus::RUNNING;
}

void TriggerService::onHalted() {
    if (service_future_.has_value()){
        trigger_client_->remove_pending_request(*service_future_);
        service_future_ = std::nullopt;
    }
}

} // namespace spot_behaviors
