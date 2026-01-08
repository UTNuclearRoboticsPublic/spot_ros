////////////////////////////////////////////////////////////////////////////////////////////
//      Title     : walk_to_pose.cpp
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

#include "spot_behaviors/walk_to_pose.hpp"

#include <set>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace spot_behaviors{

WalkToPose::WalkToPose(const std::string& name, const BT::NodeConfig& config, tf2_ros::Buffer::SharedPtr tf_buffer) :
    BT::StatefulActionNode(name, config),
    NodeBehaviorBase(name, tf_buffer)
{
    navigation_action_client_ = rclcpp_action::create_client<spot_msgs::action::WalkTo>(this, "/spot_driver/walk_to");
}

BT::PortsList WalkToPose::providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::PoseStamped::SharedPtr>("target_pose"),
        BT::InputPort<std::string>("speed_profile", "NORMAL", "The type of movement speed desired. Options are [SLOW, NORMAL, FAST, CURRENT]"),
        BT::InputPort<double>("trans_err_threshold"),
        BT::InputPort<double>("rot_err_threshold")
    };
}

BT::NodeStatus WalkToPose::onStart() {
    static const std::set<std::string> valid_profiles{"SLOW", "NORMAL", "FAST", "CURRENT"};

    // Check to see that the server and input are in place
    if (!navigation_action_client_->wait_for_action_server(std::chrono::seconds(10))){
        RCLCPP_ERROR(get_logger(), "/navigate_to_pose action server not available, aborting call for Spot navigation");
        return BT::NodeStatus::FAILURE;
    }

    auto target_pose_expected = getInput<geometry_msgs::msg::PoseStamped::SharedPtr>("target_pose");
    if (!target_pose_expected.has_value() || !target_pose_expected.value()){
        RCLCPP_ERROR(get_logger(), "\"target_pose\" blackboard entry not available, aborting call for Spot navigation");
        RCLCPP_ERROR(get_logger(), "Error message: %s", target_pose_expected.error().c_str());
        return BT::NodeStatus::FAILURE;
    }

    BT::Expected<double> trans_err_threshold_exp = getInput<double>("trans_err_threshold");
    if (trans_err_threshold_exp.has_value()) {
        trans_err_threshold_ = trans_err_threshold_exp.value();
        RCLCPP_INFO_STREAM(get_logger(),
            "Using translational error threshold: " << trans_err_threshold_);
    }
    
    BT::Expected<double> rot_err_threshold_exp = getInput<double>("rot_err_threshold");
    if (rot_err_threshold_exp.has_value()) {
        rot_err_threshold_ = rot_err_threshold_exp.value();
        RCLCPP_INFO_STREAM(get_logger(),
            "Using rotational error threshold: " << rot_err_threshold_);
    }

    const std::string profile = getInput<std::string>("speed_profile").value();
    if (!valid_profiles.contains(profile)) {
        RCLCPP_ERROR(get_logger(), "Invalid speed profile '%s'. Options are [SLOW, NORMAL, FAST, CURRENT]", profile.c_str());
        return BT::NodeStatus::FAILURE;
    }

    // Record the target for goal checking later
    target_pose_ = *target_pose_expected.value();

    // Generate the action request
    spot_msgs::action::WalkTo::Goal navigation_goal;
    navigation_goal.target_pose = target_pose_;
    navigation_goal.maximum_movement_time = 10.0;
    
    if (profile == "SLOW") {
        navigation_goal.max_vel.linear.x = 0.5;
        navigation_goal.max_vel.linear.y = 0.3;
        navigation_goal.max_vel.angular.z = 0.45;
    } else if (profile == "NORMAL") {
        navigation_goal.max_vel.linear.x = 0.8;
        navigation_goal.max_vel.linear.y = 0.5;
        navigation_goal.max_vel.angular.z = 0.6;
    } else if (profile == "FAST") {
        navigation_goal.max_vel.linear.x = 2.0;
        navigation_goal.max_vel.linear.y = 2.0;
        navigation_goal.max_vel.angular.z = 1.3;
    }
    // If profile is "CURRENT" then we leave max_vel as all zeros and the driver will use the current max_vel parameter settings

    goal_handle_future_ = navigation_action_client_->async_send_goal(navigation_goal);
    request_time_point_ = now();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WalkToPose::onRunning() {
    // Check to see if we're still waiting for the request to be processed
    if (goal_handle_future_.valid()){
        auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future_, std::chrono::milliseconds(5));
        switch (result){
            case rclcpp::FutureReturnCode::TIMEOUT:{
                const rclcpp::Duration duration = now() - request_time_point_; 
                if (duration > std::chrono::seconds(1)){
                    RCLCPP_ERROR(get_logger(), "Timed out waiting for response from /spot_driver/walk_to server. Aborting");
                    navigation_action_client_->async_cancel_all_goals();
                    goal_handle_future_ = decltype(goal_handle_future_){};
                    return BT::NodeStatus::FAILURE;
                }
                else return BT::NodeStatus::RUNNING;
            }

            case rclcpp::FutureReturnCode::INTERRUPTED:
                RCLCPP_ERROR(get_logger(), "Request interrupted waiting for response from /spot_driver/walk_to server. Aborting");
                goal_handle_future_ = decltype(goal_handle_future_){};
                return BT::NodeStatus::FAILURE;

            case rclcpp::FutureReturnCode::SUCCESS:
                RCLCPP_INFO(get_logger(), "WalkTo goal was acknowledged");
                goal_handle_ = goal_handle_future_.get();
                if (goal_handle_ == nullptr){
                    RCLCPP_INFO(get_logger(), "WalkTo goal was rejected");
                }else{
                    RCLCPP_INFO(get_logger(), "WalkTo goal was accepted");
                }
                goal_handle_future_ = decltype(goal_handle_future_){};
                return (goal_handle_ == nullptr) ? BT::NodeStatus::FAILURE : BT::NodeStatus::RUNNING;
        }
    }

    // If we have a goal, check its status
    if (goal_handle_ != nullptr){
        rclcpp::spin_some(this->get_node_base_interface());
        auto goal_status = goal_handle_->get_status();
        switch (goal_status){
            case action_msgs::msg::GoalStatus::STATUS_CANCELING:
            case action_msgs::msg::GoalStatus::STATUS_ACCEPTED:
            case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
                return BT::NodeStatus::RUNNING;

            case action_msgs::msg::GoalStatus::STATUS_UNKNOWN:
                RCLCPP_WARN(get_logger(), "Navigate action returned status UNKNOWN, reporting failure");
                [[fallthrough]];
            case action_msgs::msg::GoalStatus::STATUS_ABORTED:
            case action_msgs::msg::GoalStatus::STATUS_CANCELED:
                RCLCPP_WARN(get_logger(), "Navigate action failed with status %s, checking if we're within threshold distance of the goal", goal_status == action_msgs::msg::GoalStatus::STATUS_ABORTED ? "ABORTED" : "CANCELLED");
                goal_handle_.reset();
                if (checkGoal()) {
                    RCLCPP_INFO(get_logger(), "Robot is within acceptable tolerance of the goal pose, reporting success");
                    return BT::NodeStatus::SUCCESS;
                }
                else RCLCPP_WARN(get_logger(), "Robot is too far from target pose, reporting failure");
                return BT::NodeStatus::FAILURE;

            case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
                RCLCPP_INFO(get_logger(), "WalkToPose: Navigate Action completed successfully");
                goal_handle_.reset();
                return BT::NodeStatus::SUCCESS;
        }
        RCLCPP_ERROR(get_logger(), "WalkToPose action returned unknown status code \"%d\", reporting failure", +goal_status);
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_ERROR(get_logger(), "WalkToPose has no active action or action request. This should never happen");
    return BT::NodeStatus::FAILURE;
}

void WalkToPose::onHalted() {
    if (goal_handle_future_.valid() || goal_handle_ != nullptr){
        auto cancel_future = navigation_action_client_->async_cancel_all_goals();
        auto response = rclcpp::spin_until_future_complete(this->get_node_base_interface(), cancel_future, std::chrono::seconds(1));
        if (response == rclcpp::FutureReturnCode::TIMEOUT || response == rclcpp::FutureReturnCode::INTERRUPTED){
            RCLCPP_FATAL(get_logger(), "Unable to cancel WalkToPose action request. Robot may move unexpectedly!!!");
        }
    }
}

bool WalkToPose::checkGoal() const {
    // Let the most up-to-date TF data arrive
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // Get the robot pose in the target frame
    std::optional<Eigen::Isometry3d> robot_pose = getFramePose("base_footprint", target_pose_.header.frame_id).value();
    if (!robot_pose.has_value()) {
        return false;
    }

    // Convert the target pose to Eigen
    Eigen::Isometry3d target_pose;
    tf2::fromMsg(target_pose_.pose, target_pose);

    // Get the relative error
    const double translation_error = (robot_pose->translation() - target_pose.translation()).norm();
    const double rotation_error = std::abs(Eigen::Quaterniond(robot_pose->rotation()).angularDistance(Eigen::Quaterniond(target_pose.rotation())));

    RCLCPP_INFO(get_logger(), "Translation error: %.2f | Rotation error: %.2f", translation_error, rotation_error);

    // Compare to the threshold values (hard coded for now - will change to parameters later)
    return translation_error < trans_err_threshold_ && rotation_error < rot_err_threshold_;
}

} // namespace spot_behaviors
