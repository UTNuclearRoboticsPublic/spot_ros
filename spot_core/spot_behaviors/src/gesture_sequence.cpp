#include "spot_behaviors/gesture_sequence.hpp"

namespace spot_behaviors {

GestureSequence::GestureSequence(const std::string& name, const BT::NodeConfiguration& config, tf2_ros::Buffer::SharedPtr tf_buffer):
    BT::StatefulActionNode(name, config),
    NodeBehaviorBase(name, tf_buffer)
{
    gesture_sequence_client_ = this->create_client<spot_msgs::srv::GestureSequence>("/spot_driver/gesture_sequence");
}

BT::PortsList GestureSequence::providedPorts() {
    return {
        BT::InputPort<std::shared_ptr<std::vector<spot_msgs::msg::Gesture>>>("gesture_sequence"),
        BT::InputPort<std::string>("gesture_mode")
    };
}


BT::NodeStatus GestureSequence::onStart() {
    if (!gesture_sequence_client_->wait_for_service(std::chrono::seconds(2))){
        RCLCPP_ERROR(get_logger(), "Server \"%s\" not found, aborting", gesture_sequence_client_->get_service_name());
        return BT::NodeStatus::FAILURE;
    }

    if (service_future_.has_value()){
        RCLCPP_ERROR(get_logger(), "Gesture sequence service called with an existing request. This should never happen");
        return BT::NodeStatus::FAILURE;
    }

    request_ = std::make_shared<spot_msgs::srv::GestureSequence::Request>();
    request_->gesture_mode = getInput<std::string>("gesture_mode").value_or("gesture_sequence");
    request_->gesture_sequence = *getInput<std::shared_ptr<std::vector<spot_msgs::msg::Gesture>>>("gesture_sequence").value();

    
    if (request_->gesture_sequence.empty() && request_->gesture_mode.empty()) {
        RCLCPP_WARN(get_logger(), "Received empty gesture sequence. No gestures will be executed.");
    }
    else if (request_->gesture_mode.empty()) {
        RCLCPP_INFO(get_logger(), "No sequence_mode specified: Defaulting to sending gesture sequence with %zu gestures.", request_->gesture_sequence.size());
    }
    else {
        RCLCPP_INFO(get_logger(), "Initiating %s.", request_->gesture_mode.c_str());

    }

    // The following computes the total length of the gesture sequence
    sequence_duration_ = 5.0f;
    for (const auto& gesture : request_->gesture_sequence) {
        sequence_duration_ += gesture.pose_duration;
    }

    service_future_ = gesture_sequence_client_->async_send_request(request_);
    request_timestamp_ = now();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GestureSequence::onRunning() {
    if (!service_future_.has_value()) {
        RCLCPP_WARN(get_logger(), "Spot gesture sequence behavior running without an active request. This should never happen.");
        return BT::NodeStatus::FAILURE;
    }

    auto status = rclcpp::spin_until_future_complete(this->get_node_base_interface(), service_future_.value(), std::chrono::milliseconds(5));

    if (status == rclcpp::FutureReturnCode::SUCCESS) {
        auto result = service_future_->get();
        service_future_ = std::nullopt;
        return result->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    if (status == rclcpp::FutureReturnCode::TIMEOUT) {
        double elapsed_seconds = (now() - request_timestamp_).seconds();
        if (elapsed_seconds > sequence_duration_) {
            RCLCPP_ERROR(get_logger(), "Timed out waiting for gesture sequence server to respond. Aborting behavior.");
            gesture_sequence_client_->remove_pending_request(service_future_.value());
            service_future_ = std::nullopt;
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    if (status == rclcpp::FutureReturnCode::INTERRUPTED) {
        RCLCPP_WARN(get_logger(), "Spot gesture sequence request interrupted. Reporting failure.");
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::FAILURE; // Fallback case
}

void GestureSequence::onHalted() {
    if (service_future_.has_value()) {
        gesture_sequence_client_->remove_pending_request(*service_future_);
        service_future_ = std::nullopt;
    }
}

} // namespace spot_behaviors
