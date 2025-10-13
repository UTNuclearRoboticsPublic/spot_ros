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
      BT::OutputPort<bool>("is_holding")
  };
}

BT::NodeStatus GetGripperHoldingState::tick() {

  // Refresh ROS and get the holding state
  rclcpp::spin_some(node_);

  if (!gripper_state_received_) {
    RCLCPP_WARN(node_->get_logger(),
                "No gripper state received yet from topic %s. Could be due to network delay.", gripper_state_topic_name_);
    return BT::NodeStatus::FAILURE;
  }


  setOutput("is_holding", is_holding_);
  return BT::NodeStatus::SUCCESS;
}

} // namespace spot_behaviors

