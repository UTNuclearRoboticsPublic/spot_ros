#include "spot_behaviors/get_gripper_holding_state.hpp"

namespace spot_behaviors {

GetGripperHoldingState::GetGripperHoldingState(const std::string &name, const BT::NodeConfig &config)
    : BT::SyncActionNode(name, config) 
{
  node_ = rclcpp::Node::make_shared(name);

  // Subscribe to the gripper state topic
  gripper_state_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
  gripper_state_topic_name_,
  10,
  [this](const std_msgs::msg::Bool::SharedPtr msg) {
    is_holding_ = msg->data;
    gripper_state_received_ = true;
  });

}

BT::PortsList GetGripperHoldingState::providedPorts() {
  return {
      BT::InputPort<double>("timeout_secs"),
      BT::OutputPort<bool>("is_holding")
  };
}

BT::NodeStatus GetGripperHoldingState::tick() {
    // Retrieve optional timeout parameter
    BT::Expected<double> timeout_exp = getInput<double>("timeout_secs");
    double timeout_sec = default_timeout_sec_;

    if (!timeout_exp) {
        RCLCPP_WARN(
            node_->get_logger(),
            "Timeout not specified to GetGripperHoldingState behavior. Using default %.2f s.",
            default_timeout_sec_
        );
    } else {
        timeout_sec = timeout_exp.value();
    }

    // Define timeout and rate
    const rclcpp::Time start_time = node_->now();
    const rclcpp::Duration timeout_duration = rclcpp::Duration::from_seconds(timeout_sec);
    rclcpp::Rate rate(20.0);  // 20 Hz = 50 ms loop period

    // Refresh until timeout or gripper begins holding
    while (rclcpp::ok() && (node_->now() - start_time < timeout_duration)) {
        rclcpp::spin_some(node_);

        if (gripper_state_received_ && is_holding_) {
            break;
        }

        rate.sleep();
    }

    // If no message was ever received, fail
    if (!gripper_state_received_) {
        RCLCPP_WARN(
            node_->get_logger(),
            "Timeout (%.2f s) waiting for gripper holding state from topic '%s'.",
            timeout_sec, gripper_state_topic_name_
        );
        return BT::NodeStatus::FAILURE;
    }

    // Otherwise, output the current state (true or false)
    setOutput("is_holding", is_holding_);
    return BT::NodeStatus::SUCCESS;
}

} // namespace spot_behaviors

