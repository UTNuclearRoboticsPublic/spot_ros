#include "spot_behaviors/get_stow_state.hpp"

namespace spot_behaviors {

GetStowState::GetStowState(const std::string &name, const BT::NodeConfig &config)
    : BT::SyncActionNode(name, config) 
{
  node_ = rclcpp::Node::make_shared(name);

  // Subscribe to the stow state topic
  stow_state_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
  stow_state_topic_name_,
  10,
  [this](const std_msgs::msg::Bool::SharedPtr msg) {
    is_stowed_ = msg->data;
    stow_state_received_ = true;
  });

}

BT::PortsList GetStowState::providedPorts() {
  return {
      BT::InputPort<double>("timeout_secs"),
      BT::OutputPort<bool>("is_stowed")
  };
}

BT::NodeStatus GetStowState::tick() {
    // Retrieve optional timeout parameter
    BT::Expected<double> timeout_exp = getInput<double>("timeout_secs");
    double timeout_sec = default_timeout_sec_;

    if (!timeout_exp) {
        RCLCPP_WARN(
            node_->get_logger(),
            "Timeout not specified to GetStowState behavior. Using default %.2f s.",
            default_timeout_sec_
        );
    } else {
        timeout_sec = timeout_exp.value();
    }

    // Define timeout and rate
    const rclcpp::Time start_time = node_->now();
    const rclcpp::Duration timeout_duration = rclcpp::Duration::from_seconds(timeout_sec);
    rclcpp::Rate rate(20.0);  // 20 Hz = 50 ms loop period

    // Refresh until timeout or stowed
    while (rclcpp::ok() && (node_->now() - start_time < timeout_duration)) {
        rclcpp::spin_some(node_);

        if (stow_state_received_ && is_stowed_) {
	    break;
        }

        rate.sleep();
    }

    // Timeout reached
    if (!stow_state_received_) {
        RCLCPP_WARN(
            node_->get_logger(),
            "Timeout (%.2f s) waiting for stow state from topic '%s'.",
            timeout_sec, stow_state_topic_name_
        );
        return BT::NodeStatus::FAILURE;
    }

    // We received updates
    RCLCPP_INFO(
      node_->get_logger(),
      is_stowed_ ? "Arm is stowed." : "Arm is not stowed."
    );
    setOutput("is_stowed", is_stowed_);
    return BT::NodeStatus::SUCCESS;

}


} // namespace spot_behaviors

