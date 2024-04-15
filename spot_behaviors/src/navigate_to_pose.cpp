#include "spot_behaviors/navigate_to_pose.hpp"

namespace spot_behaviors{

NavigateToPose::NavigateToPose(const std::string& name, const BT::NodeConfig& config) :
    BT::StatefulActionNode(name, config),
    node_(std::make_shared<rclcpp::Node>(name, "spot_behaviors"))
{
    navigation_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, "/navigate_to_pose");
}

BT::PortsList NavigateToPose::providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose")
    };
}

BT::NodeStatus NavigateToPose::onStart() {
    // Check to see that the server and input are in place
    if (!navigation_action_client_->wait_for_action_server(std::chrono::milliseconds(500))){
        RCLCPP_ERROR(node_->get_logger(), "/navigate_to_pose action server not available, aborting call for Spot navigation");
        return BT::NodeStatus::FAILURE;
    }

    BT::Expected<geometry_msgs::msg::PoseStamped> target_pose_expected = getInput<geometry_msgs::msg::PoseStamped>("target_pose");
    if (!target_pose_expected.has_value()){
        RCLCPP_ERROR(node_->get_logger(), "\"target_pose\" blackboard entry not available, aborting call for Spot navigation");
        RCLCPP_ERROR(node_->get_logger(), "Error message: %s", target_pose_expected.error().c_str());
        return BT::NodeStatus::FAILURE;
    }

    // Generate the action request
    nav2_msgs::action::NavigateToPose::Goal navigation_goal;
    navigation_goal.behavior_tree = ""; // To be tested, I want to see if this just uses the default
    navigation_goal.pose = target_pose_expected.value();

    goal_handle_future_ = navigation_action_client_->async_send_goal(navigation_goal);
    request_time_point_ = node_->now();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToPose::onRunning() {
    // Check to see if we're still waiting for the request to be processed
    if (goal_handle_future_.valid()){
        auto result = rclcpp::spin_until_future_complete(node_, goal_handle_future_, std::chrono::milliseconds(5));
        switch (result){
            case rclcpp::FutureReturnCode::TIMEOUT:{
                const rclcpp::Duration duration = node_->now() - request_time_point_; 
                if (duration > std::chrono::seconds(1)){
                    RCLCPP_ERROR(node_->get_logger(), "Timed out waiting for response from /navigate_to_pose server. Aborting");
                    navigation_action_client_->async_cancel_all_goals();
                    goal_handle_future_ = decltype(goal_handle_future_){};
                    return BT::NodeStatus::FAILURE;
                }
                else return BT::NodeStatus::RUNNING;
            }

            case rclcpp::FutureReturnCode::INTERRUPTED:
                RCLCPP_ERROR(node_->get_logger(), "Request interrupted waiting for response from /navigate_to_pose server. Aborting");
                goal_handle_future_ = decltype(goal_handle_future_){};
                return BT::NodeStatus::FAILURE;

            case rclcpp::FutureReturnCode::SUCCESS:
                RCLCPP_INFO(node_->get_logger(), "Navigation goal was acknowledged");
                goal_handle_ = goal_handle_future_.get();
                goal_handle_future_ = decltype(goal_handle_future_){};
                return (goal_handle_ == nullptr) ? BT::NodeStatus::FAILURE : BT::NodeStatus::RUNNING;
        }
    }

    // If we have a goal, check its status
    if (goal_handle_ != nullptr){
        auto goal_status = goal_handle_->get_status();
        switch (goal_status){
            case action_msgs::msg::GoalStatus::STATUS_CANCELING:
            case action_msgs::msg::GoalStatus::STATUS_ACCEPTED:
            case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
                return BT::NodeStatus::RUNNING;

            case action_msgs::msg::GoalStatus::STATUS_UNKNOWN:
                RCLCPP_WARN(node_->get_logger(), "Navigate action returned status UNKNOWN, reporting failure");
                [[fallthrough]];
            case action_msgs::msg::GoalStatus::STATUS_ABORTED:
            case action_msgs::msg::GoalStatus::STATUS_CANCELED:
                RCLCPP_WARN(node_->get_logger(), "Navigate action failed");
                goal_handle_.reset();
                return BT::NodeStatus::FAILURE;

            case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
                RCLCPP_INFO(node_->get_logger(), "NavigateToPose: Navigate Action completed successfully");
                goal_handle_.reset();
                return BT::NodeStatus::SUCCESS;
        }
        RCLCPP_ERROR(node_->get_logger(), "NavigateToPose action returned unknown status code \"%d\", reporting failure", +goal_status);
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_ERROR(node_->get_logger(), "NavigateToPose has no active action or action request. This should never happen");
    return BT::NodeStatus::FAILURE;
}

void NavigateToPose::onHalted() {
    if (goal_handle_future_.valid() || goal_handle_ != nullptr){
        auto cancel_future = navigation_action_client_->async_cancel_all_goals();
        auto response = rclcpp::spin_until_future_complete(node_, cancel_future, std::chrono::seconds(1));
        if (response == rclcpp::FutureReturnCode::TIMEOUT || response == rclcpp::FutureReturnCode::INTERRUPTED){
            RCLCPP_FATAL(node_->get_logger(), "Unable to cancel NavigateToPose action request. Robot may move unexpectedly!!!");
        }
    }
}

} // namespace spot_behaviors
