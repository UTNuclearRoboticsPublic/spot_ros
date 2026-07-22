#include "spot_navigation/spot_controller.hpp"

#include <map>
#include <pluginlib/class_list_macros.hpp>

namespace spot_navigation {

void SpotController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    node_ = parent;
    plugin_name_ = name;
    tf_buffer_ = tf_buffer;
    costmap_ = costmap_ros;

    // Create the spot driver action client
    auto node = node_.lock();
    walk_to_client_ = rclcpp_action::create_client<spot_msgs::action::WalkTo>(node, "/spot_driver/walk_to");
    if (!walk_to_client_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(get_logger(), "Unable to contact WalkTo server");
        throw std::runtime_error("Unable to contact WalkTo server");
    } else {
        RCLCPP_INFO(get_logger(), "Connected to WalkTo server");
    }
    
    // Declare parameters and parameter update function
    max_vx_     = node->declare_parameter<double>(plugin_name_ + ".max_vel.x");
    max_vy_     = node->declare_parameter<double>(plugin_name_ + ".max_vel.y");
    max_vtheta_ = node->declare_parameter<double>(plugin_name_ + ".max_vel.theta");
    lookahead_dist_ = node->declare_parameter<double>(plugin_name_ + ".lookahead_dist");

    if (!node->get_parameter<double>("controller_frequency", controller_frequency_)) {
        RCLCPP_ERROR(get_logger(), "Unable to determine controller frequency");
        throw std::runtime_error("Unable to determine controller frequency");
    }

    params_map_ = std::map<std::string, double*>(
        {
            {plugin_name_ + ".max_vel.x", &max_vx_},
            {plugin_name_ + ".max_vel.y", &max_vy_},
            {plugin_name_ + ".max_vel.theta", &max_vtheta_},
            {plugin_name_ + ".lookahead_dist", &lookahead_dist_}
        }
    );

    params_callback_handle_ = node->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) -> rcl_interfaces::msg::SetParametersResult {
            for (const auto& param : params) {
                if (param.get_parameter_value().get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) continue;
                if (params_map_.contains(param.get_name())) {
                    *params_map_[param.get_name()] = param.as_double();
                    RCLCPP_INFO(get_logger(), "Updating %s to %.2f", param.get_name().c_str(), param.as_double());
                }
            }
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            return result;
        }
    );
}

void SpotController::cleanup() {
    if (walk_to_goal_handle_) {
        walk_to_client_->async_cancel_goal(walk_to_goal_handle_);
        walk_to_goal_handle_.reset();
    }
    walk_to_client_.reset();
}

void SpotController::activate() {
    // Nothing to activate
}

void SpotController::deactivate() {
    // Nothing to deactivate
}

geometry_msgs::msg::TwistStamped SpotController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped& pose,
    const geometry_msgs::msg::Twist& velocity,
    nav2_core::GoalChecker * goal_checker)
{
    // Since we don't control the robot via cmd_vel, we always return an empty twist message
    geometry_msgs::msg::TwistStamped null_twist;
    null_twist.header = pose.header;

    if (std::string err; !tf_buffer_->canTransform(global_path_.header.frame_id, pose.header.frame_id, pose.header.stamp, rclcpp::Duration::from_seconds(0.3), &err)) {
        RCLCPP_WARN(get_logger(), "Unable to transform robot pose to global frame: %s", err.c_str());
        return null_twist;
    }

    // Determine robot location in same frame as the path
    const geometry_msgs::msg::PoseStamped robot_pose_in_world = tf_buffer_->transform(
        pose,
        global_path_.header.frame_id,
        tf2::durationFromSec(0.3)
    );

    // If we've reached, don't send a new goal
    if (walk_to_goal_handle_ && goal_checker->isGoalReached(robot_pose_in_world.pose, global_path_.poses.back().pose, velocity)) {
        walk_to_client_->async_cancel_goal(walk_to_goal_handle_);
        walk_to_goal_handle_.reset();
        return null_twist;
    }

    auto pose_dist = [](const geometry_msgs::msg::PoseStamped& robot_pose, const geometry_msgs::msg::PoseStamped& path_pose) -> double {
        return std::sqrt(std::pow(robot_pose.pose.position.x - path_pose.pose.position.x, 2) + std::pow(robot_pose.pose.position.y - path_pose.pose.position.y, 2));
    };
    
    bool inside_region = false;
    double path_length = 0.0;
    std::optional<geometry_msgs::msg::PoseStamped> target_pose;
    
    // Find the point on the path that is the required distance away from either the robot or the last executed waypoint, whichever is lower
    for (std::size_t pose_idx = last_pose_index_-1; pose_idx < global_path_.poses.size(); pose_idx++) {
        path_length += pose_dist(global_path_.poses[pose_idx], global_path_.poses[pose_idx-1]);
        const double robot_dist = pose_dist(global_path_.poses[pose_idx], robot_pose_in_world);
        if (robot_dist < lookahead_dist_) {
            inside_region = true;
    // We don't control the robot via cmd_vel
            target_pose = global_path_.poses[pose_idx];
            if (path_length >= lookahead_dist_) {
                // We exceeded our max path length
                break;
            }
        } else if (inside_region) {
            // We exited the robot lookahead region
            break;
        }
    }

    // If the target pose is not set here, that means that the robot is far away
    // from where we expect it to be if it's making progress. We halt the robot
    // and wait for a new plan
    if (!target_pose && walk_to_goal_handle_) {
        walk_to_client_->async_cancel_goal(walk_to_goal_handle_);
        walk_to_goal_handle_.reset();
        RCLCPP_WARN(get_logger(), "Made negative progress - waiting for new plan");
        return null_twist;
    }

    auto walk_to_goal = std::make_shared<spot_msgs::action::WalkTo::Goal>();
    walk_to_goal->target_pose = target_pose.value();
    walk_to_goal->maximum_movement_time = 2*(1.0/controller_frequency_);
    walk_to_goal->max_vel.linear.x = max_vx_;
    walk_to_goal->max_vel.linear.y = max_vy_;
    walk_to_goal->max_vel.angular.z = max_vtheta_;

    rclcpp_action::Client<spot_msgs::action::WalkTo>::SendGoalOptions opts;
    opts.goal_response_callback = [this](const rclcpp_action::Client<spot_msgs::action::WalkTo>::GoalHandle::SharedPtr& goal_handle) {
        walk_to_goal_handle_ = goal_handle;
    };
    walk_to_client_->async_send_goal(*walk_to_goal, opts);

    return null_twist;
}

void SpotController::setPlan(const nav_msgs::msg::Path& path) {
    global_path_ = path;
    last_pose_index_ = 1;
}

void SpotController::setSpeedLimit(const double& speed_limit, const bool& percentage) {
    const double current_speed_limit = std::sqrt(std::pow(max_vx_, 2) + std::pow(max_vy_, 2));
    const double ratio = percentage ? speed_limit/100.0 : speed_limit / current_speed_limit;
    max_vx_     *= ratio;
    max_vy_     *= ratio;
    max_vtheta_ *= ratio;

    auto node = node_.lock();
    node->set_parameters({
        rclcpp::Parameter(plugin_name_ + ".max_vel.x", max_vx_),
        rclcpp::Parameter(plugin_name_ + ".max_vel.y", max_vy_),
        rclcpp::Parameter(plugin_name_ + ".max_vel.theta", max_vtheta_),
    });
}

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(spot_navigation::SpotController, nav2_core::Controller);

} // namespace spot_navigation
