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

MoveHandToPose::MoveHandToPose(const std::string& name, const BT::NodeConfiguration& config, tf2_ros::Buffer::SharedPtr tf_buffer):
    BT::StatefulActionNode(name, config),
    NodeBehaviorBase(name, tf_buffer)
{
    move_group_action_client_ = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(this, "/spot_moveit/move_action");
    bosdyn_action_client_ = rclcpp_action::create_client<spot_msgs::action::ArmCartesianCommand>(this, "/spot_manipulation_driver/arm_cartesian_command");
    max_planning_time_ = this->declare_parameter<double>("manipulation.max_planning_time", 5.0);
    planning_group_    = this->declare_parameter<std::string>("manipulation.planning_group", "arm");
    max_velocity_scaling_factor_ = this->declare_parameter<double>("manipulation.max_velocity_scaling_factor", 0.1);
    max_acceleration_scaling_factor_ = this->declare_parameter<double>("manipulation.max_acceleration_scaling_factor", 0.1);
}

BT::PortsList MoveHandToPose::providedPorts(){
    return {
        BT::InputPort<geometry_msgs::msg::PoseStamped::SharedPtr>("target_pose"),
        BT::InputPort<std::string>("target_link"),
        BT::InputPort<std::string>("planning_group"),
        BT::InputPort<std::string>("backend", "moveit", "Which action service to call for the motion. Options are [moveit, bosdyn]"),
        BT::InputPort<std::string>("pipeline", "The planning pipeline to use for planning"),
        BT::InputPort<std::string>("planner", "The planner to use"),
        BT::InputPort<float>("planning_timeout", "The time to wait for the planner to compute in seconds")
    };
}

BT::NodeStatus MoveHandToPose::onStart() {
    const std::string backend = getInput<std::string>("backend").value_or("moveit");
    if (backend != "moveit" && backend != "bosdyn") {
        RCLCPP_ERROR(get_logger(), "Unrecognized back end for MoveHandToPose: [%s]", backend.c_str());
        return BT::NodeStatus::FAILURE;
    }

    // Make sure the action client is up and running
    if (backend == "moveit" && !move_group_action_client_->wait_for_action_server(std::chrono::seconds(10))){
        RCLCPP_ERROR(get_logger(), "Move Group action client did not respond, aborting MoveHandToPose behavior");
        return BT::NodeStatus::FAILURE;
    }else if (backend == "bosdyn" && !bosdyn_action_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(get_logger(), "Boston Dynamics action client did not respond, aborting MoveHandToPose behavior");
        return BT::NodeStatus::FAILURE;
    }else{
        RCLCPP_INFO(get_logger(), "Action client found");
    }

    // Retrieve the target pose from blackboard
    auto target_pose_expected = getInput<geometry_msgs::msg::PoseStamped::SharedPtr>("target_pose");
    if (!target_pose_expected.has_value()){
        RCLCPP_ERROR(
            get_logger(), 
            "Unable to retrieve target pose from blackboard, aborting MoveHandToPose behavior\nReason: %s", 
            target_pose_expected.error().c_str()
        );
        return BT::NodeStatus::FAILURE;
    }
    const geometry_msgs::msg::PoseStamped& target_pose = *target_pose_expected.value();

    // Check to see what frame we want to define the pose for
    const std::string target_link = getInput<std::string>("target_link").value_or("arm0_hand");

    // Generate the action server goal
    if (backend == "moveit") {
        moveit_msgs::action::MoveGroup::Goal move_group_goal;
        move_group_goal.planning_options.plan_only = false;
        move_group_goal.planning_options.replan = false;
        move_group_goal.request.allowed_planning_time = getInput<float>("planning_timeout").value_or(max_planning_time_);
        move_group_goal.request.max_velocity_scaling_factor = max_velocity_scaling_factor_;
        move_group_goal.request.max_acceleration_scaling_factor = max_acceleration_scaling_factor_;
        move_group_goal.request.goal_constraints.push_back(
            kinematic_constraints::constructGoalConstraints(target_link, target_pose)
        );
        move_group_goal.request.group_name = getInput<std::string>("planning_group").value_or("arm");
        move_group_goal.request.workspace_parameters.header.frame_id = target_pose.header.frame_id;
        move_group_goal.request.workspace_parameters.header.stamp = now();
        move_group_goal.request.workspace_parameters.min_corner.x = -1e9;
        move_group_goal.request.workspace_parameters.min_corner.y = -1e9;
        move_group_goal.request.workspace_parameters.min_corner.z = -1e9;
        move_group_goal.request.workspace_parameters.max_corner.x = +1e9;
        move_group_goal.request.workspace_parameters.max_corner.y = +1e9;
        move_group_goal.request.workspace_parameters.max_corner.z = +1e9;

        if (std::string pipeline; getInput("pipeline", pipeline)) {
            move_group_goal.request.pipeline_id = pipeline;
        }

        if (std::string planner; getInput("planner", planner)) {
            move_group_goal.request.planner_id = planner;
        }

        // LIN planner often fails at high velocities
        if (move_group_goal.request.pipeline_id == "pilz_industrial_motion_planner" && move_group_goal.request.planner_id == "LIN") {
            move_group_goal.request.max_velocity_scaling_factor = std::max(move_group_goal.request.max_velocity_scaling_factor, 0.2);
        }

        // Request the motion
        RCLCPP_INFO(get_logger(), "Sending move group goal to action server");
        move_group_response_future_ = move_group_action_client_->async_send_goal(move_group_goal);
        request_timestamp_ = move_group_goal.request.workspace_parameters.header.stamp;

    } else if (backend == "bosdyn") {
        // TODO: Add preferred joint configuration

        spot_msgs::action::ArmCartesianCommand::Goal arm_command;
        arm_command.header = target_pose.header;
        arm_command.tool_frame = target_link;
        arm_command.waypoints.push_back(target_pose.pose);
        arm_command.timestamps.push_back(0.0);      // TODO: parameterize
        arm_command.max_linear_velocity    =  0.15; // TODO: parameterize
        arm_command.max_acceleration       = 10.0;  // TODO: parameterize
        arm_command.max_pos_tracking_error =  1.2;
        arm_command.force_remain_near_current_joint_configuration = true; // TODO: parameterize
        arm_command.x_axis_mode = arm_command.AXIS_MODE_POSITION;
        arm_command.y_axis_mode = arm_command.AXIS_MODE_POSITION;
        arm_command.z_axis_mode = arm_command.AXIS_MODE_POSITION;
        arm_command.rx_axis_mode = arm_command.AXIS_MODE_POSITION;
        arm_command.rx_axis_mode = arm_command.AXIS_MODE_POSITION;
        arm_command.rx_axis_mode = arm_command.AXIS_MODE_POSITION;

        RCLCPP_INFO(get_logger(), "Sending arm cartesian command to Boston Dynamics action server");
        bosdyn_response_future_ = bosdyn_action_client_->async_send_goal(arm_command);
        request_timestamp_ = now();
    }

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveHandToPose::onRunning() {
    if (!rclcpp::ok()){return BT::NodeStatus::FAILURE;}

    // Check to see if we're still waiting on a response from the action server
    if (move_group_response_future_.valid() || bosdyn_response_future_.valid()){
        const bool using_moveit = move_group_response_future_.valid();
        const std::string server_name = using_moveit ? "MoveIt" : "Boston Dynamics";
        const auto result = using_moveit ? 
                            rclcpp::spin_until_future_complete(this->get_node_base_interface(), move_group_response_future_, std::chrono::milliseconds(5)) : 
                            rclcpp::spin_until_future_complete(this->get_node_base_interface(), bosdyn_response_future_, std::chrono::milliseconds(5));

        switch (result){
            case rclcpp::FutureReturnCode::SUCCESS:
                if (using_moveit) {
                    move_group_goal_handle_ = move_group_response_future_.get();
                    move_group_response_future_ = decltype(move_group_response_future_){};
                    motion_start_time_ = now();
                } else {
                    bosdyn_goal_handle_ = bosdyn_response_future_.get();
                    bosdyn_response_future_ = decltype(bosdyn_response_future_){};
                }
                return (move_group_goal_handle_ || bosdyn_goal_handle_) ? BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE;

            case rclcpp::FutureReturnCode::TIMEOUT:{
                const double elapsed_seconds = (now() - request_timestamp_).seconds();
                if (elapsed_seconds > 2.0){
                    RCLCPP_ERROR(get_logger(), "Timed out waiting for %s action server to respond. Aborting MoveHandToPose behavior", server_name.c_str());
                    onHalted();
                    return BT::NodeStatus::FAILURE;
                }
                else return BT::NodeStatus::RUNNING;
            }

            case rclcpp::FutureReturnCode::INTERRUPTED:
                RCLCPP_WARN(get_logger(), "MoveHandToPose request interrupted. Reporting failed movement");
                onHalted();
                return BT::NodeStatus::FAILURE;
        }
    } 

    // Once we reach this part of the function, a goal handle MUST be active
    if (!move_group_goal_handle_ && !bosdyn_goal_handle_){
        RCLCPP_ERROR(get_logger(), "MoveHandToPose has no active action or action request. This should never happen");
        return BT::NodeStatus::FAILURE;
    }

    // Check if the action is ongoing, or if it has concluded
    rclcpp::spin_some(this->get_node_base_interface());
    const std::string action_name = move_group_goal_handle_ ? "MoveGroup" : "ArmCartesianCommand";
    const int8_t goal_status = move_group_goal_handle_ ? move_group_goal_handle_->get_status() : bosdyn_goal_handle_->get_status();
    switch (goal_status){
        case action_msgs::msg::GoalStatus::STATUS_CANCELING:
        case action_msgs::msg::GoalStatus::STATUS_ACCEPTED:
        case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
        {
            const rclcpp::Duration elapsed_time = now() - motion_start_time_;
            if (elapsed_time.seconds() > 10.0) {
                move_group_action_client_->async_cancel_all_goals();
            }
            return BT::NodeStatus::RUNNING;
        }

        case action_msgs::msg::GoalStatus::STATUS_UNKNOWN:
            RCLCPP_WARN(get_logger(), "%s action returned status UNKNOWN, reporting failure", action_name.c_str());
            [[fallthrough]];
        case action_msgs::msg::GoalStatus::STATUS_ABORTED:
        case action_msgs::msg::GoalStatus::STATUS_CANCELED:
            RCLCPP_WARN(get_logger(), "%s action failed", action_name.c_str());
            move_group_goal_handle_.reset();
            return BT::NodeStatus::FAILURE;

        case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
            RCLCPP_INFO(get_logger(), "MoveHandToPose: %s Action complete", action_name.c_str());
            move_group_goal_handle_.reset();
            return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_ERROR(get_logger(), "%s action returned unknown status code \"%d\", reporting failure", action_name.c_str(), +goal_status);
    return BT::NodeStatus::FAILURE;
}

void MoveHandToPose::onHalted() {
    if (move_group_response_future_.valid() || move_group_goal_handle_ != nullptr){
        auto cancel_future = move_group_action_client_->async_cancel_all_goals();
        auto response = rclcpp::spin_until_future_complete(this->get_node_base_interface(), cancel_future, std::chrono::seconds(1));
        if (response == rclcpp::FutureReturnCode::TIMEOUT || response == rclcpp::FutureReturnCode::INTERRUPTED){
            RCLCPP_FATAL(get_logger(), "Unable to cancel MoveGroup action request. Robot may move unexpectedly!!!");
        }
    }

    if (bosdyn_response_future_.valid() || bosdyn_goal_handle_ != nullptr){
        auto cancel_future = bosdyn_action_client_->async_cancel_all_goals();
        auto response = rclcpp::spin_until_future_complete(this->get_node_base_interface(), cancel_future, std::chrono::seconds(1));
        if (response == rclcpp::FutureReturnCode::TIMEOUT || response == rclcpp::FutureReturnCode::INTERRUPTED){
            RCLCPP_FATAL(get_logger(), "Unable to cancel ArmCartesianCommand action request. Robot may move unexpectedly!!!");
        }
    }
}

} // namespace spot_behaviors
