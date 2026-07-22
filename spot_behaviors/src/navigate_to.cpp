#include "spot_behaviors/navigate_to.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace spot_behaviors {

NavigateTo::NavigateTo(const std::string& name, const BT::NodeConfig& config, tf2_ros::Buffer::SharedPtr tf_buffer) :
StatefulActionNode(name, config),
NodeBehaviorBase(name, tf_buffer)
{
    action_name_ = "/spot_driver/navigate_to";
    navigation_client_ = rclcpp_action::create_client<spot_msgs::action::NavigateTo>(this, action_name_);
}

BT::PortsList NavigateTo::providedPorts() {
    return {
        BT::InputPort<std::string>("upload_path", "Absolute path to map_directory, which is downloaded from tablet controller"),
        BT::InputPort<std::string>("waypoint_id", "The ID of the pose to navigate to"),
        BT::InputPort<bool>("initial_localization_fiducial", "Whether or not to use fiducials for initial localization"),
        BT::InputPort<std::string>("initial_localization_waypoint", "Initial waypoint id at which to trigger localization "),
        BT::InputPort<float>("client_timeout", 5.0f, "Time in seconds to wait for the navigation server to respond"),
        BT::InputPort<float>("navigation_timeout", std::numeric_limits<float>::infinity(), "Time in seconds to wait for the navigation action to comlete"),
    };
}

BT::NodeStatus NavigateTo::onStart() {    
    getInput("client_timeout", client_timeout_);
    getInput("navigation_timeout", navigation_timeout_);
    
    request_start_time_ = now();
    if (!navigation_client_->wait_for_action_server(std::chrono::duration<float>(client_timeout_))) {
        RCLCPP_ERROR(get_logger(), "Did not receive a response from the \"%s\" action server within the timeout of %.1f seconds",
                                    action_name_.c_str(), client_timeout_);
    }

    // Set the goal fields from the input ports
    rclcpp_action::Client<spot_msgs::action::NavigateTo>::Goal goal;
    getInput("upload_path", goal.upload_path);
    getInput("waypoint_id", goal.waypoint_id);
    getInput("initial_localization_fiducial", goal.initial_localization_fiducial);
    getInput("initial_localization_waypoint", goal.initial_localization_waypoint);

    // Configure callbacks to set data members as the action request progresses
    rclcpp_action::Client<spot_msgs::action::NavigateTo>::SendGoalOptions opts;
    opts.goal_response_callback = [this](rclcpp_action::Client<spot_msgs::action::NavigateTo>::GoalHandle::SharedPtr goal_handle) -> void {
        goal_handle_ = goal_handle;
        response_received_ = true;
        goal_handle_future_ = {};
        navigation_start_time_ = now();
    };
    opts.result_callback = [this](const rclcpp_action::Client<spot_msgs::action::NavigateTo>::WrappedResult& result) {
        result_ = *result.result;
    };

    // Send goal and immediately return running
    goal_handle_future_ = navigation_client_->async_send_goal(goal, opts);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateTo::onRunning() {
    rclcpp::spin_some(get_node_base_interface());
    BT::NodeStatus current_status = BT::NodeStatus::RUNNING;

    if (!response_received_) {
        // We're still waiting on a response to our goal request (i.e. navigation has not started yet)
        const float elapsed_time = (now() - request_start_time_).seconds();
        if (elapsed_time > client_timeout_) {
            RCLCPP_ERROR(get_logger(), "Did not receive a response from the client within the timeout of %.1f seconds. Aborting", client_timeout_);
            current_status = BT::NodeStatus::FAILURE;
        }
    } 
    else if (!goal_handle_) {
        // We've received a response but our goal handle is still null
        RCLCPP_WARN(get_logger(), "Goal was rejected by the server");
        current_status = BT::NodeStatus::FAILURE;
    }
    
    if (!result_) {
        // We're still waiting on a result of the navigation
        const float elapsed_time = (now() - navigation_start_time_).seconds();
        if (elapsed_time > navigation_timeout_) {
            RCLCPP_ERROR(get_logger(), "Did not complete navigation within the timeout of %.1f seconds. Aborting", navigation_timeout_);
            current_status = BT::NodeStatus::FAILURE;
        }
    } 
    else {
        // We've received a result
        if (result_->success) {
            RCLCPP_INFO(get_logger(), "Navigation completed successfully");
            current_status = BT::NodeStatus::SUCCESS;
        } else {
            RCLCPP_WARN(get_logger(), "Navigation failed: %s", result_->message.c_str());
            current_status = BT::NodeStatus::FAILURE;
        }
    }

    if (BT::isStatusCompleted(current_status)) {
        onHalted();
    }

    return current_status;
}

void NavigateTo::onHalted() {
    if (goal_handle_future_.valid()) {
        navigation_client_->async_cancel_all_goals();
    }
    else if (goal_handle_) {
        navigation_client_->async_cancel_goal(goal_handle_);
    }
    goal_handle_future_ = {};
    goal_handle_.reset();
    response_received_ = false;
    result_.reset();
}

} // namespace spot_behaviors
