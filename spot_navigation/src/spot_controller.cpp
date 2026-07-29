#include "spot_navigation/spot_controller.hpp"

#include <map>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace spot_navigation {

SpotController::SpotController(
    const std::string& xml_tag_name,
    const std::string& action_name,
    const BT::NodeConfiguration& bt_config
) : 
StatefulActionNode(xml_tag_name, bt_config)
{
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    tf_buffer_ = config().blackboard->get<tf2_ros::Buffer::SharedPtr>("tf_buffer");

    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

    walk_to_client_ = rclcpp_action::create_client<spot_msgs::action::WalkTo>(node_, "/spot_driver/walk_to");
    target_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/spot_nav/spot_controller/target_pose", rclcpp::QoS{1}.transient_local());

    // Declare parameters and parameter update function
    walk_to_goal_.max_vel.linear.x  = node_->get_parameter("spot_controller.max_vel.x").as_double();
    walk_to_goal_.max_vel.linear.y  = node_->get_parameter("spot_controller.max_vel.y").as_double();
    walk_to_goal_.max_vel.angular.z = node_->get_parameter("spot_controller.max_vel.theta").as_double();
    lookahead_dist_ = node_->get_parameter("spot_controller.lookahead_dist").as_double();
    controller_frequency_ = node_->get_parameter("spot_controller.frequency").as_double();

    params_map_ = std::map<std::string, double*>(
        {
            {"spot_controller.max_vel.x"     , &walk_to_goal_.max_vel.linear.x},
            {"spot_controller.max_vel.y"     , &walk_to_goal_.max_vel.linear.y},
            {"spot_controller.max_vel.theta" , &walk_to_goal_.max_vel.angular.z},
            {"spot_controller.lookahead_dist", &lookahead_dist_},
            {"spot_controller.frequency"     , &controller_frequency_}
        }
    );

    params_callback_handle_ = node_->add_on_set_parameters_callback(
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

    goal_options_.goal_response_callback = [this](const rclcpp_action::Client<spot_msgs::action::WalkTo>::GoalHandle::SharedPtr& goal_handle) {
        walk_to_goal_handle_ = goal_handle;
        movement_start_time_ = goal_handle->get_goal_stamp();
    };

    goal_options_.feedback_callback = [this](
        rclcpp_action::Client<spot_msgs::action::WalkTo>::GoalHandle::ConstSharedPtr goal_handle,
        spot_msgs::action::WalkTo::Feedback::ConstSharedPtr feedback) 
    {
        if (walk_to_goal_handle_ && (goal_handle->get_goal_id() == walk_to_goal_handle_->get_goal_id())) {
            walk_to_feedback_ = feedback;   
        }
    };

    goal_options_.result_callback = [this](const rclcpp_action::Client<spot_msgs::action::WalkTo>::GoalHandle::WrappedResult& result) {
        // Filter out old goals
        if (walk_to_goal_handle_ && result.goal_id == walk_to_goal_handle_->get_goal_id()) {
            walk_to_success_ = result.result->success;
        }
    };
}

BT::PortsList SpotController::providedPorts() {
    return {
        BT::InputPort<nav_msgs::msg::Path>("path", "The global path to follow")
    };
};

BT::NodeStatus SpotController::onStart() {
    // Get the goal from the blackboard
    try{
        getUpdatedPath();
    } catch (BT::RuntimeError& e) {
        RCLCPP_ERROR(get_logger(), "Failed to retrieve global plan from blackboard: %s", e.what());
        return BT::NodeStatus::FAILURE;
    }

    // Determine the goal to send the robot to based on the path
    if (auto goal_pose = calculateNextGoal(); goal_pose) {
        sendNewGoal(goal_pose.value());
    } else {
        RCLCPP_ERROR(get_logger(), "Unable to determine determine valid goal pose");
        return BT::NodeStatus::FAILURE;
    }

    // Send the goal
    walk_to_success_.reset();
    walk_to_client_->async_send_goal(walk_to_goal_, goal_options_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SpotController::onRunning() {
    executor_.spin_some();

    // If we recevied a new plan, the requisite time has passed, or we've finished this segment, then start a new motion
    const double elapsed_time = (node_->now() - request_start_time_).seconds();
    const bool enough_time_has_passed = elapsed_time > 1.0/controller_frequency_;
    const bool needs_to_continue = 
        !isTerminalGoal() 
        && walk_to_feedback_
        && (
            walk_to_feedback_->status_enum == walk_to_feedback_->STATUS_STOPPING || 
            walk_to_feedback_->status_enum == walk_to_feedback_->STATUS_STOPPED
        );
    if (getUpdatedPath() || enough_time_has_passed || needs_to_continue) {
        auto target_pose = calculateNextGoal();
        
        // If the target pose is not set here, that means that the robot is far away
        // from where we expect it to be if it's making progress. We halt the robot
        if (!target_pose) {
            RCLCPP_WARN(get_logger(), "Unable to determine a new target pose");
            onHalted();
            return BT::NodeStatus::FAILURE;
        }

        sendNewGoal(target_pose.value());
    }

    // Check to see if we're done
    if (isTerminalGoal() && walk_to_success_.has_value()) {
        if (walk_to_success_.value()) {
            walk_to_goal_handle_.reset();
            onHalted();
            return BT::NodeStatus::SUCCESS;
        } else if (!getUpdatedPath()) {
            onHalted();
            return BT::NodeStatus::FAILURE;
        } else {
            return BT::NodeStatus::RUNNING;
        }
    }

    return BT::NodeStatus::RUNNING;
}

void SpotController::onHalted() {
    if (walk_to_goal_handle_ && !walk_to_success_) {
        RCLCPP_INFO(get_logger(), "Halted, cancelling goal");
        walk_to_client_->async_cancel_goal(walk_to_goal_handle_);
    }
    walk_to_goal_handle_.reset();
    walk_to_success_.reset();
    last_pose_index_ = 1;
    global_path_ = nav_msgs::msg::Path{};
}

std::optional<geometry_msgs::msg::PoseStamped> SpotController::calculateNextGoal() {
    // Determine robot location in same frame as the path
    geometry_msgs::msg::PoseStamped robot_pose;
    robot_pose.header.frame_id = "base_footprint";
    if (std::string err; !tf_buffer_->canTransform(global_path_.header.frame_id, robot_pose.header.frame_id, robot_pose.header.stamp, rclcpp::Duration::from_seconds(0.3), &err)) {
        RCLCPP_WARN(get_logger(), "Unable to transform robot pose to global frame: %s", err.c_str());
        return std::nullopt;
    }
    const geometry_msgs::msg::PoseStamped robot_pose_in_world = tf_buffer_->transform(
        robot_pose,
        global_path_.header.frame_id,
        tf2::durationFromSec(0.3)
    );

    // Function to determine distance from robot pose to another pose on the path
    auto pose_dist = [](const geometry_msgs::msg::PoseStamped& robot_pose, const geometry_msgs::msg::PoseStamped& path_pose) -> double {
        return std::sqrt(std::pow(robot_pose.pose.position.x - path_pose.pose.position.x, 2) + std::pow(robot_pose.pose.position.y - path_pose.pose.position.y, 2));
    };
    
    bool inside_region = false;
    double path_length = 0.0;
    std::optional<geometry_msgs::msg::PoseStamped> target_pose;
    
    // Find the point on the path that is the required distance away from either the robot or the last executed waypoint, whichever is lower
    for (std::size_t pose_idx = last_pose_index_; pose_idx < global_path_.poses.size(); pose_idx++) {
        path_length += pose_dist(global_path_.poses[pose_idx], global_path_.poses[pose_idx-1]);
        const double robot_dist = pose_dist(global_path_.poses[pose_idx], robot_pose_in_world);
        if (robot_dist < lookahead_dist_) {
            inside_region = true;
            target_pose = global_path_.poses[pose_idx];
            last_pose_index_ = pose_idx;
            if (path_length >= lookahead_dist_) {
                // We exceeded our max path length
                break;
            }
        } else if (inside_region) {
            // We exited the robot lookahead region
            break;
        }
    }

    if (!target_pose) RCLCPP_WARN(get_logger(), "Made negative progress - aborting movement");
    return target_pose;
}

bool SpotController::getUpdatedPath() {
    const nav_msgs::msg::Path global_path = getInput<nav_msgs::msg::Path>("path").value();
    if (global_path != global_path_) {
        RCLCPP_INFO(get_logger(), "Got updated path");
        last_pose_index_ = 1;
        global_path_ = global_path;
        return true;
    }
    return false;
}

bool SpotController::isTerminalGoal() const {
    if (global_path_.poses.empty()) return false;
    return walk_to_goal_.target_pose == global_path_.poses.back();
}

void SpotController::sendNewGoal(const geometry_msgs::msg::PoseStamped& target_pose) {
    walk_to_goal_handle_.reset();
    
    walk_to_goal_.target_pose = target_pose;
    walk_to_goal_.maximum_movement_time = 10*(1.0/controller_frequency_);
    walk_to_success_.reset();
    walk_to_feedback_.reset();
    request_start_time_ = node_->now();
    RCLCPP_INFO(get_logger(), "Sending new goal");
    walk_to_client_->async_send_goal(walk_to_goal_, goal_options_);
    target_pose_pub_->publish(target_pose);
}

} // namespace spot_navigation

#include <behaviortree_cpp_v3/bt_factory.h>
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder = [](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<spot_navigation::SpotController>(name, "spot_controller", config);
    };

    factory.registerBuilder<spot_navigation::SpotController>("SpotController", builder);
}
