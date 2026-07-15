////////////////////////////////////////////////////////////////////////////////////////////
//      Title     : check_arm_stowed.hpp
//      Project   : spot_ros
////////////////////////////////////////////////////////////////////////////////////////////

#pragma once
#include <thread>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>

#include "spot_behaviors/node_behavior_base.hpp"
#include "spot_msgs/msg/manipulator_stow_state.hpp"

namespace spot_behaviors {

class CheckArmStowed : public BT::SyncActionNode, public NodeBehaviorBase {
public:
    CheckArmStowed(const std::string& name, const BT::NodeConfiguration& config, tf2_ros::Buffer::SharedPtr tf_buffer);

    static inline BT::PortsList providedPorts() { return {}; };

    // Returns SUCCESS if the arm is stowed, FAILURE otherwise
    BT::NodeStatus tick() override;

private:
    // Subscriber to data
    rclcpp::Subscription<spot_msgs::msg::ManipulatorStowState>::SharedPtr manipulator_sub_;

    // Record the last stow state
    std::optional<bool> arm_is_stowed_;

    // Record the last stow state obtained from the message
    void manipulatorStateCallback(spot_msgs::msg::ManipulatorStowState::UniquePtr msg);
};

} // namespace spot_behaviors
