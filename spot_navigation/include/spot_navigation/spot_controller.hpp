#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// Conditional for supporting Humble and prior
#if __has_include(<behaviortree_cpp_v3/action_node.h>)
#include <behaviortree_cpp_v3/action_node.h>
#else
#include <behaviortree_cpp/action_node.h>
#endif

#include "spot_msgs/action/walk_to.hpp"

namespace spot_navigation {

class SpotController : public BT::StatefulActionNode {
public:
    SpotController(
        const std::string& xml_tag_name,
        const std::string& action_name,
        const BT::NodeConfiguration& bt_config
    );

    ~SpotController() override = default;
    
    // One input port - the global path [nav_msgs::msg::Path]
    static BT::PortsList providedPorts();

    // Retrieve the global path from the blackboard and set the robot
    // towards the first waypoint in the path
    BT::NodeStatus onStart() final;

    // Check for any of the following conditions to send a new pose to the robot:
    //  - A new global path on blackboard
    //  - Sufficient time has passed since the last goal was issued
    //  - The walk_to feedback reports that the robot is stopping and we
    //    are not on the final waypoint of the global path
    //
    // Otherwise check for action completion
    BT::NodeStatus onRunning() final;
    void onHalted() final;

private:
    rclcpp::Logger get_logger() const {return rclcpp::get_logger("SpotController");}

    // ROS interface
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;

    // Global path from the planner server
    nav_msgs::msg::Path global_path_;

    // Action-Server interface
    spot_msgs::action::WalkTo::Goal walk_to_goal_;
    spot_msgs::action::WalkTo::Feedback::ConstSharedPtr walk_to_feedback_;
    rclcpp_action::Client<spot_msgs::action::WalkTo>::SharedPtr walk_to_client_;
    rclcpp_action::Client<spot_msgs::action::WalkTo>::GoalHandle::SharedPtr walk_to_goal_handle_;
    rclcpp_action::Client<spot_msgs::action::WalkTo>::SendGoalOptions goal_options_;
    rclcpp::Time movement_start_time_;
    rclcpp::Time request_start_time_;
    
    // Bookkeeping for planning
    std::size_t last_pose_index_;
    std::optional<bool> walk_to_success_;
    bool goal_handle_received_;

    // Update global_path_ from the blackboard
    // Returns true if the path is not the same as last time. False otherwise
    bool getUpdatedPath();

    // Calculate where the robot should go based on the current robot position,
    // the global plan, and the previous state of the planner
    std::optional<geometry_msgs::msg::PoseStamped> calculateNextGoal();

    // Whether or not the current goal is the final one in the global plan
    bool isTerminalGoal() const;

    // Send a new goal and reset goal metrics
    void sendNewGoal(const geometry_msgs::msg::PoseStamped& target_pose);

    // === Parameters === //
    double lookahead_dist_;
    double controller_frequency_;
    
    std::thread params_thread_;
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
    std::map<std::string, double*> params_map_;
    
}; // class SpotController

} // namespace spot_navigation
