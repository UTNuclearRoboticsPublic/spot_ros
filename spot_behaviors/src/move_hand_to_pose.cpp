////////////////////////////////////////////////////////////////////////////////////////////
//      Title     : move_arm_to_pose.cpp
//      Project   : spot_ros
//      Copyright : Copyright© The University of Texas at Austin, 2024. All rights reserved.
//                
//          All files within this directory are subject to the following, unless an alternative
//          license is explicitly included within the text of each file.
//
//          This software and documentation constitute an unpublished work
//          and contain valuable trade secrets and proprietary information
//          belonging to the University. None of the foregoing material may be
//          copied or duplicated or disclosed without the express, written
//          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
//          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
//          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
//          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
//          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
//          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
//          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
//          University be liable for incidental, special, indirect, direct or
//          consequential damages or loss of profits, interruption of business,
//          or related expenses which may arise from use of software or documentation,
//          including but not limited to those resulting from defects in software
//          and/or documentation, or loss or inaccuracy of data of any kind.
//
////////////////////////////////////////////////////////////////////////////////////////////

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/kinematic_constraints/utils.h>

#include "spot_behaviors/move_hand_to_pose.hpp"

namespace spot_behaviors{

MoveHandToPose::MoveHandToPose(const std::string& name, const BT::NodeConfiguration& config):
    BT::StatefulActionNode(name, config),
    node_(std::make_shared<rclcpp::Node>(name, "spot_behaviors"))
{
    move_group_action_client_ = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(node_, "/move_action");
    max_planning_time_ = node_->declare_parameter<double>("manipulation.max_planning_time", 5.0);
    planning_group_    = node_->declare_parameter<std::string>("manipulation.planning_group", "arm");
}

BT::PortsList MoveHandToPose::providedPorts(){
    return {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose"),
        BT::InputPort<std::string>("target_frame"),
        BT::InputPort<std::string>("planning_group")
    };
}

BT::NodeStatus MoveHandToPose::onStart() {
    // Make sure the action client is up and running
    RCLCPP_INFO(node_->get_logger(), "Waiting for action server");
    if (!move_group_action_client_->wait_for_action_server(std::chrono::seconds(10))){
        RCLCPP_ERROR(node_->get_logger(), "Move Group action client did not respond, aborting MoveHandToPose behavior");
        return BT::NodeStatus::FAILURE;
    }else{
        RCLCPP_INFO(node_->get_logger(), "Move Group action client found");
    }

    // Retrieve the target pose from blackboard
    BT::Expected<geometry_msgs::msg::PoseStamped> target_pose_expected = getInput<geometry_msgs::msg::PoseStamped>("target_pose");
    if (!target_pose_expected.has_value()){
        RCLCPP_ERROR(
            node_->get_logger(), 
            "Unable to retrieve target pose from blackboard, aborting MoveHandToPose behavior\nReason: %s", 
            target_pose_expected.error().c_str()
        );
        return BT::NodeStatus::FAILURE;
    }
    const geometry_msgs::msg::PoseStamped& target_pose = target_pose_expected.value();

    // Check to see what frame we want to define the pose for
    const std::string target_frame = getInput<std::string>("target_frame").value_or("arm0_hand");
    RCLCPP_INFO(node_->get_logger(), "Target frame: %s", target_frame.c_str());

    // Generate the action server goal
    moveit_msgs::action::MoveGroup::Goal move_group_goal;
    move_group_goal.planning_options.plan_only = false;
    move_group_goal.planning_options.replan = false;
    move_group_goal.request.allowed_planning_time = max_planning_time_;
    move_group_goal.request.max_velocity_scaling_factor = 0.1;
    move_group_goal.request.goal_constraints.push_back(
        kinematic_constraints::constructGoalConstraints(target_frame, target_pose)
    );
    move_group_goal.request.group_name = getInput<std::string>("planning_group").value_or("arm");
    move_group_goal.request.workspace_parameters.header.frame_id = "base_link";
    move_group_goal.request.workspace_parameters.header.stamp = node_->now();
    move_group_goal.request.workspace_parameters.min_corner.x = -1e9;
    move_group_goal.request.workspace_parameters.min_corner.y = -1e9;
    move_group_goal.request.workspace_parameters.min_corner.z = -1e9;
    move_group_goal.request.workspace_parameters.max_corner.x = +1e9;
    move_group_goal.request.workspace_parameters.max_corner.y = +1e9;
    move_group_goal.request.workspace_parameters.max_corner.z = +1e9;

    // Request the motion
    move_group_response_future_ = move_group_action_client_->async_send_goal(move_group_goal);
    request_timestamp_ = move_group_goal.request.workspace_parameters.header.stamp;

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveHandToPose::onRunning() {
    if (!rclcpp::ok()){return BT::NodeStatus::FAILURE;}

    // Check to see if we're still waiting on a response from the action server
    if (move_group_response_future_.valid()){
        const auto result = rclcpp::spin_until_future_complete(node_, move_group_response_future_, std::chrono::milliseconds(5));
        switch (result){
            case rclcpp::FutureReturnCode::SUCCESS:
                move_group_goal_handle_ = move_group_response_future_.get();
                move_group_response_future_ = decltype(move_group_response_future_){};
                return move_group_goal_handle_ ? BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE;

            case rclcpp::FutureReturnCode::TIMEOUT:{
                const double elapsed_seconds = (node_->now() - request_timestamp_).seconds();
                if (elapsed_seconds > 2.0){
                    RCLCPP_ERROR(node_->get_logger(), "Timed out waiting for MoveGroup action server to respond. Aborting MoveHandToPose behavior");
                    move_group_action_client_->async_cancel_all_goals();
                    return BT::NodeStatus::FAILURE;
                }
                else return BT::NodeStatus::RUNNING;
            }

            case rclcpp::FutureReturnCode::INTERRUPTED:
                RCLCPP_WARN(node_->get_logger(), "MoveHandToPose MoveGroup request interrupted. Reporting failed movement");
                return BT::NodeStatus::FAILURE;
        }
    } else if (!move_group_goal_handle_){
        RCLCPP_ERROR(node_->get_logger(), "MoveHandToPose has no active action or action request. This should never happen");
        return BT::NodeStatus::FAILURE;
    }

    // Check if the action is ongoing, or if it has concluded
    rclcpp::spin_some(node_);
    const int8_t goal_status = move_group_goal_handle_->get_status();
    switch (goal_status){
        case action_msgs::msg::GoalStatus::STATUS_CANCELING:
        case action_msgs::msg::GoalStatus::STATUS_ACCEPTED:
        case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
            return BT::NodeStatus::RUNNING;

        case action_msgs::msg::GoalStatus::STATUS_UNKNOWN:
            RCLCPP_WARN(node_->get_logger(), "MoveGroup action returned status UNKNOWN, reporting failure");
            [[fallthrough]];
        case action_msgs::msg::GoalStatus::STATUS_ABORTED:
        case action_msgs::msg::GoalStatus::STATUS_CANCELED:
            RCLCPP_WARN(node_->get_logger(), "MoveGroup action failed");
            move_group_goal_handle_.reset();
            return BT::NodeStatus::FAILURE;

        case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
            RCLCPP_INFO(node_->get_logger(), "MoveHandToPose: MoveGroup Action complete");
            move_group_goal_handle_.reset();
            return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_ERROR(node_->get_logger(), "MoveGroup action returned unknown status code \"%d\", reporting failure", +goal_status);
    return BT::NodeStatus::FAILURE;
}

void MoveHandToPose::onHalted() {
    if (move_group_response_future_.valid() || move_group_goal_handle_ != nullptr){
        auto cancel_future = move_group_action_client_->async_cancel_all_goals();
        auto response = rclcpp::spin_until_future_complete(node_, cancel_future, std::chrono::seconds(1));
        if (response == rclcpp::FutureReturnCode::TIMEOUT || response == rclcpp::FutureReturnCode::INTERRUPTED){
            RCLCPP_FATAL(node_->get_logger(), "Unable to cancel MoveGroup action request. Robot may move unexpectedly!!!");
        }
    }
}

} // namespace spot_behaviors
