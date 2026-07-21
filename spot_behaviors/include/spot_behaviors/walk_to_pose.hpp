////////////////////////////////////////////////////////////////////////////////////////////
//      Title     : walk_to_pose.hpp
//      Project   : spot_ros
////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Note: This files is necessary because spot_behaviors uses behavior_tree.CPP V4 while
 *       the ros nav2 package uses behavior_tree.CPP V3. If this ever changes in the 
 *       future it is recommended that this file be deleted or archived in favor of the
 *       official release (May 15th, 2024)
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <behaviortree_cpp/action_node.h>

#include "spot_msgs/action/walk_to.hpp"
#include "spot_behaviors/node_behavior_base.hpp"

namespace spot_behaviors{

class WalkToPose : public BT::StatefulActionNode, public NodeBehaviorBase {
public:
    WalkToPose(const std::string& name, const BT::NodeConfig& config, tf2_ros::Buffer::SharedPtr tf_buffer);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    void onHalted() override;

private:
    static const inline std::string action_server_name_ = "/spot_driver/walk_to";

    rclcpp_action::Client<spot_msgs::action::WalkTo>::SharedPtr navigation_action_client_;
    std::shared_future<rclcpp_action::ClientGoalHandle<spot_msgs::action::WalkTo>::SharedPtr> goal_handle_future_;
    rclcpp_action::ClientGoalHandle<spot_msgs::action::WalkTo>::SharedPtr goal_handle_;
    rclcpp::Time request_time_point_{};

    geometry_msgs::msg::PoseStamped target_pose_;
    double trans_err_threshold_ = 0.25;
    double rot_err_threshold_ = 0.25;
    bool checkGoal() const;
};
    
} // namespace spot_behaviors
