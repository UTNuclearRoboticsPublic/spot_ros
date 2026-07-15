////////////////////////////////////////////////////////////////////////////////////////////
//      Title     : move_arm_to_pose.hpp
//      Project   : spot_ros
////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include "spot_behaviors/node_behavior_base.hpp"
#include "spot_msgs/action/arm_cartesian_command.hpp"

namespace spot_behaviors{

class MoveHandToPose : public BT::StatefulActionNode, public NodeBehaviorBase {
public:
    MoveHandToPose(const std::string& name, const BT::NodeConfiguration& config, tf2_ros::Buffer::SharedPtr tf_buffer);

    /** We accept x input ports - TODO 
     *  - target_pose: geometry_msgs::msg::PoseStamped 
     */
    static BT::PortsList providedPorts();

    /** 
     * Make the the motion request. 
     * @return RUNNING if the action server is available, FAILURE otherwise
     */
    BT::NodeStatus onStart() override;

    /**
     * Check the status of an existing request
     * @return RUNNING if request is not yet complete, FAILURE if no requset has
     *         been made, SUCCESS/FAILURE if the request was completed  
     */
    BT::NodeStatus onRunning() override;

    /**
     * If an there is an existing request, cancel it 
     */
    void onHalted() override;

protected:
    // Parameters (default values are provided at parameter declaration)
    double max_planning_time_{};
    std::string planning_group_{};
    double max_velocity_scaling_factor_{};
    double max_acceleration_scaling_factor_{};
    rclcpp::Time motion_start_time_;

    // Action client
    rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SharedPtr move_group_action_client_;
    rclcpp_action::Client<spot_msgs::action::ArmCartesianCommand>::SharedPtr bosdyn_action_client_;

    // Action client future handle - only used while waiting for a request to be accepted or jejected
    std::shared_future<rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::SharedPtr> move_group_response_future_;
    std::shared_future<rclcpp_action::ClientGoalHandle<spot_msgs::action::ArmCartesianCommand>::SharedPtr> bosdyn_response_future_;
    rclcpp::Time request_timestamp_{};

    // Action client goal handle - nullptr if no request is active
    rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::SharedPtr move_group_goal_handle_;
    rclcpp_action::ClientGoalHandle<spot_msgs::action::ArmCartesianCommand>::SharedPtr bosdyn_goal_handle_;

    template<typename ActionType>
    BT::NodeStatus checkRequestStatus(std::shared_future<typename rclcpp_action::ClientGoalHandle<ActionType>::SharedPtr> shared_future);
};

} // namespace spot_behaviors
