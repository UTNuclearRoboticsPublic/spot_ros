#include "spot_behaviors/toggle_payload.hpp"

namespace spot_behaviors {
    
TogglePayload::TogglePayload(const std::string& name, const BT::NodeConfig& config, tf2_ros::Buffer::SharedPtr tf_buffer):
BT::StatefulActionNode(name, config),
NodeBehaviorBase(name, tf_buffer)
{
    payload_client_ = create_client<spot_msgs::srv::TogglePayload>("/spot_driver/toggle_payload");
}

BT::PortsList TogglePayload::providedPorts() {
    return {
        BT::InputPort<std::string>("guid", "[optional] The guid of the payload to toggle"),
        BT::InputPort<std::string>("name", "[optional] The name of the payload to toggle"),
        BT::InputPort<bool>("attached", "Whether the payload should be attached or detached after this operation"),
        BT::InputPort<float>("timeout", 2.0f, "Timeout for request in seconds")
    };
}

BT::NodeStatus TogglePayload::onStart() {
    const std::string guid = getInput<std::string>("guid").value_or("");
    const std::string name = getInput<std::string>("name").value_or("");

    getInput("timeout", timeout_);
    request_start_time_ = now();

    if (!payload_client_->wait_for_service(std::chrono::duration<float>(timeout_))) {
        RCLCPP_ERROR(get_logger(), "Unable to contact client [%s] within %.2f seconds. Aborting", payload_client_->get_service_name(), timeout_);
        return BT::NodeStatus::FAILURE;
    }

    auto request = std::make_shared<spot_msgs::srv::TogglePayload::Request>();

    if (!getInput("attached", request->attached)) {
        RCLCPP_ERROR(get_logger(), "Missing required input [attached]. Aborting");
        return BT::NodeStatus::FAILURE;
    }
    getInput("guid", request->guid);
    getInput("name", request->name);
    
    service_future_ = payload_client_->async_send_request(request);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TogglePayload::onRunning() {
    if (!service_future_) {
        RCLCPP_ERROR(get_logger(), "Running called without an active request. This should never happen");
        return BT::NodeStatus::FAILURE;
    }

    rclcpp::FutureReturnCode status = rclcpp::spin_until_future_complete(this->get_node_base_interface(), service_future_.value(), std::chrono::milliseconds(5));
    switch (status) {
        case rclcpp::FutureReturnCode::SUCCESS: {
            spot_msgs::srv::TogglePayload::Response::SharedPtr response = service_future_->get();
            onHalted();
            if (response->success) {
                return BT::NodeStatus::SUCCESS;
            } else {
                RCLCPP_WARN(get_logger(), "Failed to toggle payload: %s", response->message.c_str());
                return BT::NodeStatus::FAILURE;
            }
        }

        case rclcpp::FutureReturnCode::INTERRUPTED:
            onHalted();
            return BT::NodeStatus::FAILURE;

        default:
        case rclcpp::FutureReturnCode::TIMEOUT: {
            const float elapsed_time = (now() - request_start_time_).seconds();
            if (elapsed_time > timeout_) {
                RCLCPP_ERROR(get_logger(), "Timed out waiting for server to respond. Aborting");
                onHalted();
                return BT::NodeStatus::FAILURE;
            } else {
                return BT::NodeStatus::RUNNING;
            }
        }
    }
}

void TogglePayload::onHalted() {
    service_future_.reset(); 
}

} // namespace spot_behaviors
