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
      BT::OutputPort<bool>("is_stowed")
  };
}

BT::NodeStatus GetStowState::tick() {

  // Refresh ROS and get the stowed state
  rclcpp::spin_some(node_);

  if (!stow_state_received_) {
    RCLCPP_WARN(node_->get_logger(),
                "No stow state received yet from topic %s. Could be due to network delay.", stow_state_topic_name_);
    return BT::NodeStatus::FAILURE;
  }


  setOutput("is_stowed", is_stowed_);
  return BT::NodeStatus::SUCCESS;
}

} // namespace spot_behaviors

