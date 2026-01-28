#include <tf2_eigen/tf2_eigen.hpp>
#include "spot_behaviors/stable_joint_motion.hpp"
#include <moveit/kinematic_constraints/utils.h>

namespace spot_behaviors {

StableJointMotion::StableJointMotion(
    const std::string& name,
    const BT::NodeConfiguration& config,
    tf2_ros::Buffer::SharedPtr tf_buffer) : 
StatefulActionNode(name, config),
NodeBehaviorBase(name, tf_buffer)
{
    moveit_plan_client_ = create_client<moveit_msgs::srv::GetMotionPlan>("/spot_moveit/plan_kinematic_path");

}

BT::PortsList StableJointMotion::providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::PoseStamped::SharedPtr>("goal_pose", "The poses to move the arm through"),
        BT::InputPort<std::string>("target_link", "The link on the robot for which the path is to be executed"),
        BT::InputPort<double>("max_planning_time", 10.0, "The maximum allowable time before abandoning the planning request and returning FAILURE")
    };
}

BT::NodeStatus StableJointMotion::onStart() {
    bosdyn_action_client_ = rclcpp_action::create_client<spot_msgs::action::ArmCartesianCommand>(shared_from_this(), "/spot_manipulation_driver/arm_cartesian_command");
    
    if (!robot_model_loader_) {
        robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(shared_from_this());
        robot_model_ = robot_model_loader_->getModel();
        robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
    }
    
    if (!getInput("target_link", target_link_)) {
        RCLCPP_ERROR(get_logger(), "Unable to read required input 'target_link'");
        return BT::NodeStatus::FAILURE;
    }

    if (!getInput("waypoints", waypoints_) || !waypoints_) {
        RCLCPP_ERROR(get_logger(), "Unable to read required input 'waypoints'");
        return BT::NodeStatus::FAILURE;
    }

    if (!getInput("max_plannning_time", max_planning_time_)) {
        RCLCPP_ERROR(get_logger(), "Unable to read required input 'max_planning_time'");
        return BT::NodeStatus::FAILURE;
    }

    // Create a Cartesian path request
    auto motion_req = std::make_shared<moveit_msgs::srv::GetMotionPlan::Request>();
    auto& req = motion_req->motion_plan_request;
    req.allowed_planning_time = max_planning_time_;
    req.group_name = "arm";
    req.max_velocity_scaling_factor = 0.3; // For testing
    req.max_acceleration_scaling_factor = 0.3; // For testing
    req.start_state.is_diff = true;
    req.workspace_parameters.max_corner.x =
    req.workspace_parameters.max_corner.y =
    req.workspace_parameters.max_corner.z = 1e9;
    req.workspace_parameters.min_corner.x =
    req.workspace_parameters.min_corner.y =
    req.workspace_parameters.min_corner.z = -1e9;

    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.pose = waypoints_->poses.back();
    goal_pose.header = waypoints_->header;
    req.goal_constraints.push_back(
        kinematic_constraints::constructGoalConstraints("arm0_hand", goal_pose)
    );

    moveit_plan_future_ = moveit_plan_client_->async_send_request(motion_req);
    cartesian_request_timestamp_ = now();
    return BT::NodeStatus::SUCCESS;
}
    
BT::NodeStatus StableJointMotion::onRunning() {
    // Check if we're waiting on MoveIt to respond
    if (moveit_plan_future_) {
        return checkActiveMoveitPlanningRequest();
    }

    // Check if we're waiting on Spot to respond
    else if (bosdyn_response_future_.valid()) {
        return checkRequestStatus();
    }

    // Check if we're actively executing the plan
    else if (bosdyn_goal_handle_) {
        return checkActiveGoalStatus();
    }

    // None of the expected cases were true - return failure
    RCLCPP_ERROR(get_logger(), "Behavior is in an unspecified state! Returning failure");
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus StableJointMotion::checkActiveMoveitPlanningRequest() {
    auto status = rclcpp::spin_until_future_complete(this->get_node_base_interface(), moveit_plan_future_->future, std::chrono::milliseconds(5));
    switch (status) {
        case rclcpp::FutureReturnCode::TIMEOUT: {
            const auto max_duration = std::chrono::duration<double>(max_planning_time_ + 1);
            if (now() - cartesian_request_timestamp_ > max_duration) {
                RCLCPP_ERROR(get_logger(), "Did not get a response from the path client within the time limit, aborting MoveHandThroughPoses");
                onHalted();
                return BT::NodeStatus::FAILURE;
            }
            return BT::NodeStatus::RUNNING;
        }
    
        default:
        case rclcpp::FutureReturnCode::INTERRUPTED: {
            RCLCPP_WARN(get_logger(), "MoveHandThroughPoses path generation step interrupted, returning failure");
            onHalted();
            return BT::NodeStatus::FAILURE;
        }

        case rclcpp::FutureReturnCode::SUCCESS: {
            moveit_msgs::srv::GetMotionPlan::Response::SharedPtr resp = moveit_plan_future_->get();
            moveit_plan_future_.reset();
            if (resp->motion_plan_response.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
                RCLCPP_ERROR(get_logger(), "Unable to find a cartesian path through the poses");
                return BT::NodeStatus::FAILURE;
            }

            spot_msgs::action::ArmCartesianCommand::Goal::SharedPtr spot_arm_motion_goal = generateGenericTrajectory(resp);
            bosdyn_action_client_->async_send_goal(*spot_arm_motion_goal);
            request_timestamp_ = now();
            return BT::NodeStatus::RUNNING;
        }
    }
}

spot_msgs::action::ArmCartesianCommand::Goal::SharedPtr StableJointMotion::generateGenericTrajectory(moveit_msgs::srv::GetMotionPlan::Response::SharedPtr resp) {
    auto spot_arm_command = std::make_shared<spot_msgs::action::ArmCartesianCommand::Goal>();
    spot_arm_command->joint_waypoints = resp->motion_plan_response.trajectory.joint_trajectory;
    spot_arm_command->x_axis_mode = spot_arm_command->AXIS_MODE_POSITION;
    spot_arm_command->y_axis_mode = spot_arm_command->AXIS_MODE_POSITION;
    spot_arm_command->z_axis_mode = spot_arm_command->AXIS_MODE_POSITION;
    for (const trajectory_msgs::msg::JointTrajectoryPoint& joint_pos : spot_arm_command->joint_waypoints.points) {
        // Update the arm state
        moveit::core::JointModelGroup* arm_group = robot_model_->getJointModelGroup("arm");
        robot_state_->setJointGroupPositions(arm_group, joint_pos.positions);
        robot_state_->setJointGroupVelocities(arm_group, joint_pos.velocities);
        robot_state_->setJointGroupAccelerations(arm_group, joint_pos.accelerations);
        robot_state_->updateLinkTransforms();

        // Get the end effector position and set it in the trajectory
        const Eigen::Isometry3d ee_pose = robot_state_->getGlobalLinkTransform("arm0_hand");
        spot_arm_command->waypoints.push_back(tf2::toMsg(ee_pose));
        spot_arm_command->timestamps.push_back(rclcpp::Duration(joint_pos.time_from_start).seconds());
    }

    return spot_arm_command;
}

BT::NodeStatus StableJointMotion::checkRequestStatus() {
    rclcpp::FutureReturnCode code = rclcpp::spin_until_future_complete(shared_from_this(), bosdyn_response_future_, std::chrono::seconds(0));
    switch (code) {
        case rclcpp::FutureReturnCode::SUCCESS:
            bosdyn_response_future_ = decltype(bosdyn_response_future_){};
            bosdyn_goal_handle_ = bosdyn_response_future_.get();
            motion_start_time_ = now();
            return BT::NodeStatus::RUNNING;

        case rclcpp::FutureReturnCode::INTERRUPTED:
            RCLCPP_WARN(get_logger(), "Motion request cancelled");
            return BT::NodeStatus::FAILURE;

        case rclcpp::FutureReturnCode::TIMEOUT:{
            const float elapsed_time = (now() - request_timestamp_).seconds();
            if (elapsed_time > 5.0) {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5, "Waiting for response from Spot driver");
            }
            return BT::NodeStatus::RUNNING;
        }
    }

    // None of the expected cases were true - return failure
    RCLCPP_ERROR(get_logger(), "Behavior is in an unspecified state! Returning failure");
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus StableJointMotion::checkActiveGoalStatus() {
    // Check if the action is ongoing, or if it has concluded
    rclcpp::spin_some(this->get_node_base_interface());
    const std::string action_name = bosdyn_goal_handle_ ? "MoveGroup" : "ArmCartesianCommand";
    const int8_t goal_status = bosdyn_goal_handle_->get_status();
    
    switch (goal_status){
        case action_msgs::msg::GoalStatus::STATUS_CANCELING:
        case action_msgs::msg::GoalStatus::STATUS_ACCEPTED:
        case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
        {
            const rclcpp::Duration elapsed_time = now() - motion_start_time_;
            if (elapsed_time.seconds() > 10.0) {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5, "Waiting for Spot driver to complete motion");
            }
            return BT::NodeStatus::RUNNING;
        }

        case action_msgs::msg::GoalStatus::STATUS_UNKNOWN:
            RCLCPP_WARN(get_logger(), "%s action returned status UNKNOWN, reporting failure", action_name.c_str());
            [[fallthrough]];
        case action_msgs::msg::GoalStatus::STATUS_ABORTED:
        case action_msgs::msg::GoalStatus::STATUS_CANCELED:
            RCLCPP_WARN(get_logger(), "%s action failed", action_name.c_str());
            bosdyn_goal_handle_.reset();
            return BT::NodeStatus::FAILURE;

        case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
            RCLCPP_INFO(get_logger(), "StableJointMotion: %s Action complete", action_name.c_str());
            bosdyn_goal_handle_.reset();
            return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_ERROR(get_logger(), "%s action returned unknown status code \"%d\", reporting failure", action_name.c_str(), +goal_status);
    return BT::NodeStatus::FAILURE;
}

void StableJointMotion::onHalted() {
    if (moveit_plan_future_) {
        moveit_plan_client_->remove_pending_request(*moveit_plan_future_);
        moveit_plan_future_.reset();
    }
}

} // namespace spot_behaviors
