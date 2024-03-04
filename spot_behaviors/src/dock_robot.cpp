#include "spot_behaviors/dock_robot.hpp"

namespace spot_behaviors {

DockRobot::DockRobot(const std::string& name, const BT::NodeConfiguration& config):
    BT::StatefulActionNode(name, config),
    node_(std::make_shared<rclcpp::Node>(name+"BT"+std::to_string(node_count_++)))
{
    dock_client_ = node_->create_client<spot_msgs::srv::Dock>("/spot_driver/dock");
}

BT::PortsList DockRobot::providedPorts() {
    return {
        BT::InputPort<float>("dock_id")
    };
}

BT::NodeStatus DockRobot::onStart() {
    if (!dock_client_->wait_for_service(std::chrono::seconds(2))){
        RCLCPP_ERROR(node_->get_logger(), "Server \"%s\" not found, aborting", dock_client_->get_service_name());
        return BT::NodeStatus::FAILURE;
    }

    if (service_future_.has_value()){
        RCLCPP_ERROR(node_->get_logger(), "Dock service called with an existing dock request. This should never happen");
        return BT::NodeStatus::FAILURE;
    }

    auto request = std::make_shared<spot_msgs::srv::Dock::Request>();
    request->dock_id = getInput<int>("dock_id").value_or(520);
    RCLCPP_INFO(node_->get_logger(), "Calling dock request to Dock ID %u", request->dock_id);\
    
    service_future_ = dock_client_->async_send_request(request);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DockRobot::onRunning() {
    if (!service_future_.has_value()) return BT::NodeStatus::FAILURE;

    if (service_future_->wait_for(std::chrono::seconds(0)) == std::future_status::ready){
        spot_msgs::srv::Dock::Response::SharedPtr result = service_future_->get();
        service_future_ = std::nullopt;
        return result->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
}

void DockRobot::onHalted() {
    if (service_future_.has_value()){
        dock_client_->remove_pending_request(*service_future_);
        service_future_ = std::nullopt;
    }
}

} // namespace spot_behaviors
