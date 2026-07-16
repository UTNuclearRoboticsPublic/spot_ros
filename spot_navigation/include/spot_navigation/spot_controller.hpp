#pragma once

#include <thread>
#include <nav2_core/controller.hpp>
#include <spot_msgs/action/walk_to.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace spot_navigation {

class SpotController : public nav2_core::Controller {
public:
    SpotController() = default;
    ~SpotController() override = default;

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros
    ) final;

    void cleanup() final;
    void activate() final;
    void deactivate() final;
    void setSpeedLimit(const double& speed_limit, const bool& percentage) final;

    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped& pose,
        const geometry_msgs::msg::Twist& velocity,
        nav2_core::GoalChecker * goal_checker
    ) final;

    void setPlan(const nav_msgs::msg::Path& path) final;
private:
    std::string plugin_name_;
    rclcpp::Logger get_logger() {return rclcpp::get_logger("PurePursuitController");}

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
    nav_msgs::msg::Path global_path_;
    std::size_t last_idx_;

    rclcpp_action::Client<spot_msgs::action::WalkTo>::SharedPtr walk_to_client_;
    rclcpp_action::Client<spot_msgs::action::WalkTo>::GoalHandle::SharedPtr walk_to_goal_handle_;
    std::size_t last_pose_index_;

    // === Parameters === //
    double max_vx_;
    double max_vy_;
    double max_vtheta_;

    double lookahead_dist_;
    double controller_frequency_;
    
    std::thread params_thread_;
    rclcpp_lifecycle::LifecycleNode::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
    std::map<std::string, double*> params_map_;
}; // class SpotController

} // namespace spot_navigation
