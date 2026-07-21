////////////////////////////////////////////////////////////////////////////////////////////
//      Title     : check_hand_collision.hpp
//      Project   : spot_ros
////////////////////////////////////////////////////////////////////////////////////////////

#pragma once
#include <thread>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <behaviortree_cpp/action_node.h>

#include "spot_behaviors/node_behavior_base.hpp"

namespace spot_behaviors {

class CheckHandCollision : public BT::SyncActionNode, public NodeBehaviorBase {
public:
    CheckHandCollision(const std::string& name, const BT::NodeConfiguration& config, tf2_ros::Buffer::SharedPtr tf_buffer);

    static BT::PortsList providedPorts();

    // Returns SUCCESS the hand is not in collision, and FAILURE if it is
    BT::NodeStatus tick() override;

private:
    // Subscriber to data
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manipulator_sub_;

    // Most recently recorded data
    std::optional<bool> in_collision_;

    // Record the collision state obtained from the message
    void collisionStateCallback(std_msgs::msg::Bool::UniquePtr msg);
};

} // namespace spot_behaviors
