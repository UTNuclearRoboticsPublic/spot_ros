#include <geometry_msgs/msg/pose_array.hpp>
#include <moveit/kinematic_constraints/utils.h>
#include "spot_behaviors/execute_stable_arm_command.hpp"

namespace spot_behaviors{

ExecuteStableArmCommand::ExecuteStableArmCommand(const std::string& name, const BT::NodeConfiguration& config, tf2_ros::Buffer::SharedPtr tf_buffer):
    BT::StatefulActionNode(name, config),
    NodeBehaviorBase(name, tf_buffer)
{
    motion_action_client_ = rclcpp_action::create_client<spot_msgs::action::StableArmCommand>(this, "/spot_moveit/execute_known_stable_trajectory");
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

BT::PortsList ExecuteStableArmCommand::providedPorts() {
    return {
        BT::InputPort<trajectory_msgs::msg::JointTrajectory::SharedPtr>("joint_trajectory", "The sequence of joint states through which to move the arm"),
        BT::InputPort<geometry_msgs::msg::PoseArray::SharedPtr>("end_effector_trajectory", "The sequence of end effector poses through which to move the arm0_hand frame"),
    };
}
    
// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

BT::NodeStatus ExecuteStableArmCommand::onStart() {
    if (!motion_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_ERROR(get_logger(), "Unable to connect to action server, aborting ExecuteStableArmCommand");
        return BT::NodeStatus::FAILURE;
    }
    
    trajectory_msgs::msg::JointTrajectory::SharedPtr joint_trajectory;
    if (!getInput("joint_trajectory", joint_trajectory) || !joint_trajectory) {
        RCLCPP_ERROR(get_logger(), "Unable to read joint trajectory from blackboard, aborting ExecuteStableArmCommand");
        return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::PoseArray::SharedPtr end_effector_trajectory;
    if (!getInput("end_effector_trajectory", end_effector_trajectory) || !end_effector_trajectory) {
        RCLCPP_ERROR(get_logger(), "Unable to read end effector trajectory from blackboard, aborting ExecuteStableArmCommand");
        return BT::NodeStatus::FAILURE;
    }

    // Erase any past state
    success_.reset();
    motion_goal_handle_.reset();
    motion_goal_future_.reset();
    motion_response_future_.reset();

    spot_msgs::action::StableArmCommand::Goal goal;
    goal.joint_trajectory = *joint_trajectory;
    goal.end_effector_waypoints = *end_effector_trajectory;

    RCLCPP_INFO(get_logger(), "Requesting execution of known stable arm command");
    rclcpp_action::Client<spot_msgs::action::StableArmCommand>::SendGoalOptions opts;
    opts.goal_response_callback = [this](rclcpp_action::ClientGoalHandle<spot_msgs::action::StableArmCommand>::SharedPtr goal_handle) {
        if (goal_handle) RCLCPP_INFO(get_logger(), "Motion request was accepted by the server");
        else RCLCPP_WARN(get_logger(), "Motion request was rejected by the server");
    };
    opts.result_callback = [this](rclcpp_action::ClientGoalHandle<spot_msgs::action::StableArmCommand>::WrappedResult result) {
        RCLCPP_INFO(get_logger(), "Received result with success value %d", +result.result->success);
        success_ = result.result->success;
    };
    motion_response_future_ = motion_action_client_->async_send_goal(goal, opts);
    request_timestamp_ = now();

    return BT::NodeStatus::RUNNING;
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

BT::NodeStatus ExecuteStableArmCommand::onRunning() {
    rclcpp::spin_some(this->get_node_base_interface());
    if (success_.has_value()) {
        return success_.value() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    } else {
        return BT::NodeStatus::RUNNING;
    }
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

void ExecuteStableArmCommand::onHalted() {
    if (hasOngoingMotionRequest() || hasOngoingMotionExecution()) {
        motion_action_client_->async_cancel_all_goals();
        motion_goal_handle_.reset();
        motion_goal_future_.reset();
        motion_response_future_.reset();
        success_.reset();
    }
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

BT::NodeStatus ExecuteStableArmCommand::checkMotionRequestStatus() {
    auto status = rclcpp::spin_until_future_complete(this->get_node_base_interface(), motion_response_future_.value(), std::chrono::milliseconds(5));
    switch (status) {
        case rclcpp::FutureReturnCode::TIMEOUT: {
            const auto max_duration = std::chrono::seconds(5);
            if (now() - request_timestamp_ > max_duration) {
                RCLCPP_ERROR(get_logger(), "Did not get a response from the server within the time limit, aborting ExecuteStableArmCommand");
                halt();
                return BT::NodeStatus::FAILURE;
            }
            return BT::NodeStatus::RUNNING;
        }
    
        default:
        case rclcpp::FutureReturnCode::INTERRUPTED: {
            RCLCPP_WARN(get_logger(), "ExecuteStableArmCommand path generation step interrupted, returning failure");
            halt();
            return BT::NodeStatus::FAILURE;
        }

        case rclcpp::FutureReturnCode::SUCCESS: {
            motion_goal_handle_ = motion_response_future_->get();
            motion_goal_future_ = motion_action_client_->async_get_result(motion_goal_handle_);
            motion_response_future_.reset();
            if (!motion_goal_handle_) {
                RCLCPP_WARN(get_logger(), "Plan rejected by server");
                return BT::NodeStatus::FAILURE;
            }
            RCLCPP_INFO(get_logger(), "Motion request was accepted by the server");
            return BT::NodeStatus::RUNNING;
        }
    }
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

BT::NodeStatus ExecuteStableArmCommand::checkMotionExecutionStatus() {
    // Possibility two - goal is active and we check its status
    auto status = rclcpp::spin_until_future_complete(this->get_node_base_interface(), motion_goal_future_.value(), std::chrono::milliseconds(0));
    if (status == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(get_logger(), "Success from future!");
        return BT::NodeStatus::SUCCESS;
    }

    // rclcpp::spin_some(this->get_node_base_interface());
    const int8_t goal_status = motion_goal_handle_->get_status();
    switch (goal_status){
        case action_msgs::msg::GoalStatus::STATUS_CANCELING:
        case action_msgs::msg::GoalStatus::STATUS_ACCEPTED:
        case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
            return BT::NodeStatus::RUNNING;

        case action_msgs::msg::GoalStatus::STATUS_UNKNOWN:
            RCLCPP_WARN(get_logger(), "server returned status UNKNOWN, reporting failure");
            [[fallthrough]];
        case action_msgs::msg::GoalStatus::STATUS_ABORTED:
        case action_msgs::msg::GoalStatus::STATUS_CANCELED:
            RCLCPP_WARN(get_logger(), "Motion execution failed");
            motion_goal_handle_.reset();
            return BT::NodeStatus::FAILURE;
            
        case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Motion execution completed successfully");
            motion_goal_handle_.reset();
            return BT::NodeStatus::SUCCESS;
        }

    RCLCPP_ERROR(get_logger(), "Server returned unknown status code \"%d\", reporting failure", +goal_status);
    return BT::NodeStatus::FAILURE;
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

} // namespace spot_behaviors
