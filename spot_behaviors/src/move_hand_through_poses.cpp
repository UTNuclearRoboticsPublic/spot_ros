#include <type_traits>
#include <geometry_msgs/msg/pose_array.hpp>
#include <moveit/kinematic_constraints/utils.h>
#include "spot_behaviors/move_hand_through_poses.hpp"

namespace spot_behaviors{

MoveHandThroughPoses::MoveHandThroughPoses(const std::string& name, const BT::NodeConfiguration& config, tf2_ros::Buffer::SharedPtr tf_buffer):
    BT::StatefulActionNode(name, config),
    NodeBehaviorBase(name, tf_buffer)
{
    path_computation_client_ = this->create_client<moveit_msgs::srv::GetCartesianPath>("/spot_moveit/compute_cartesian_path");
    traj_execution_action_client_ = rclcpp_action::create_client<moveit_msgs::action::ExecuteTrajectory>(this, "/spot_moveit/execute_trajectory");
    move_group_action_client_ = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(this, "/spot_moveit/move_action");

    max_planning_time_           = this->declare_parameter<double>("manipulation.max_planning_time", 3.0);
    max_cartesian_planning_time_ = this->declare_parameter<double>("manipulation.max_cartesian_planning_time", 5.0);
    planning_group_              = this->declare_parameter<std::string>("manipulation.planning_group", "arm");
    max_velocity_scaling_factor_ = this->declare_parameter<double>("manipulation.max_velocity_scaling_factor", 0.05);
    max_end_effector_velocity_   = this->declare_parameter<double>("manipulation.max_end_effector_velocity", 0.03);
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

BT::PortsList MoveHandThroughPoses::providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::PoseArray::SharedPtr>("waypoints", "The sequence of poses through which to move the hand"),
        BT::InputPort<std::string>("target_link", "The link on the robot which should achieve the waypoints"),
        BT::InputPort<double>("position_tolerance", "Position tolerance for non-cartesian moves"),
        BT::InputPort<double>("angular_tolerance", "Angular tolerance for non-cartesian moves")
    };
}
    
// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

BT::NodeStatus MoveHandThroughPoses::onStart() {
    if (!path_computation_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(get_logger(), "Unable to connect to \"%s\" service, aborting MoveHandThroughPoses", path_computation_client_->get_service_name());
        return BT::NodeStatus::FAILURE;
    }

    if (!traj_execution_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_ERROR(get_logger(), "Unable to connect to \"\\execute_trajectory\" action server, aborting MoveHandThroughPoses");
        return BT::NodeStatus::FAILURE;
    }

    auto waypoints_expected = getInput<geometry_msgs::msg::PoseArray::SharedPtr>("waypoints");
    if (!waypoints_expected.has_value()) {
        RCLCPP_ERROR(get_logger(), "Unable to retrieve waypoints for blackboard, aborting MoveHandThroughPoses");
        return BT::NodeStatus::FAILURE;
    }
    
    waypoints_ = *(waypoints_expected.value());
    if (waypoints_.poses.empty()) {
        RCLCPP_WARN(get_logger(), "MoveHandThroughPoses received an empty list of waypoints. Reporting successful execution of all 0 waypoints");
        return BT::NodeStatus::SUCCESS;
    }
    
    last_idx_ = 0;
    next_idx_ = 0;
    path_computation_response_future_.reset();
    return BT::NodeStatus::RUNNING;
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

BT::NodeStatus MoveHandThroughPoses::onRunning() {
    if (hasOngoingPathRequest()) {
        return checkPathRequestStatus();
    }

    else if (hasOngoingMoveGroupRequest()) {
        return checkMoveGroupRequest();
    }

    else if (hasOngoingTrajectoryExecutionRequest()) {
        return checkTrajectoryExecutionStatus();
    }

    else if (makeNewPathRequest()){
        return BT::NodeStatus::RUNNING;
    }

    else {
        return BT::NodeStatus::FAILURE;
    }
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

void MoveHandThroughPoses::onHalted() {
    cancelOngoingPathRequest();
    cancelOngoingTrajectoryExecutionRequest();
    cancelOngoingTrajectoryExecution();
    cancelOngoingMoveGroupRequest();
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

bool MoveHandThroughPoses::hasOngoingPathRequest() const {
    return path_computation_response_future_.has_value();
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

BT::NodeStatus MoveHandThroughPoses::checkPathRequestStatus() {
    auto status = rclcpp::spin_until_future_complete(this->get_node_base_interface(), path_computation_response_future_->future, std::chrono::milliseconds(5));
    switch (status) {
        case rclcpp::FutureReturnCode::TIMEOUT: {
            const auto max_duration = std::chrono::milliseconds(static_cast<int>(1000*(max_planning_time_ + 5)));
            if (now() - path_computation_response_timestamp_ > max_duration) {
                RCLCPP_ERROR(get_logger(), "Did not get a response from the path client within the time limit, aborting MoveHandThroughPoses");
                cancelOngoingPathRequest();
                return BT::NodeStatus::FAILURE;
            }
            return BT::NodeStatus::RUNNING;
        }
    
        default:
        case rclcpp::FutureReturnCode::INTERRUPTED: {
            RCLCPP_WARN(get_logger(), "MoveHandThroughPoses path generation step interrupted, returning failure");
            path_computation_response_future_.reset();
            return BT::NodeStatus::FAILURE;
        }

        case rclcpp::FutureReturnCode::SUCCESS: {
            moveit_msgs::srv::GetCartesianPath_Response::SharedPtr resp = path_computation_response_future_->get();
            path_computation_response_future_.reset();
            if (resp->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
                RCLCPP_ERROR(get_logger(), "Unable to find a cartesian path through the poses, aborting");
                return BT::NodeStatus::FAILURE;
            }

            const int num_waypoints_achieved = static_cast<int>(resp->fraction*waypoints_.poses.size());
            RCLCPP_INFO(get_logger(), "Found a carteisan path for %d (%.2f%%) of the waypoints", 
                num_waypoints_achieved, 100.0*resp->fraction);
            if (num_waypoints_achieved == 0) {
                RCLCPP_INFO(get_logger(), "Making a general (non-cartesian) move_group request instead");
                return makeNewMoveGroupRequest() ? BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE;
            }
            incrementPoseIndex(num_waypoints_achieved);
            return makeNewTrajectoryExecutionRequest(resp) ? BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE;
        }
    }
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

void MoveHandThroughPoses::cancelOngoingPathRequest() {
    if (path_computation_response_future_.has_value()) {
        path_computation_client_->remove_pending_request(path_computation_response_future_.value());
        path_computation_response_future_.reset();
    }
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

bool MoveHandThroughPoses::hasOngoingTrajectoryExecutionRequest() const {
    return traj_execution_response_future_.valid() ||  traj_execution_goal_handle_ != nullptr;
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

BT::NodeStatus MoveHandThroughPoses::checkTrajectoryExecutionStatus() {
    // Possibility one - waiting for goal to be accepted by the action server
    if (traj_execution_response_future_.valid()) {
        auto status = rclcpp::spin_until_future_complete(this->get_node_base_interface(), traj_execution_response_future_, std::chrono::milliseconds(5));
        switch (status) {
            case rclcpp::FutureReturnCode::TIMEOUT: {
                const auto max_duration = std::chrono::seconds(5);
                if (now() - traj_execution_request_timestamp_ > max_duration) {
                    RCLCPP_ERROR(get_logger(), "Did not get a response from the TrajectoryExecution server within the time limit, aborting MoveHandThroughPoses");
                    cancelOngoingTrajectoryExecutionRequest();
                    return BT::NodeStatus::FAILURE;
                }
                return BT::NodeStatus::RUNNING;
            }

            case rclcpp::FutureReturnCode::INTERRUPTED: {
                RCLCPP_WARN(get_logger(), "TrajectoryEexcution request was interrupted, reporting failure");
                abortPoseIncrement();
                traj_execution_response_future_ = decltype(traj_execution_response_future_){};
                return BT::NodeStatus::FAILURE;
            }

            case rclcpp::FutureReturnCode::SUCCESS: {
                traj_execution_goal_handle_ = traj_execution_response_future_.get();
                traj_execution_response_future_ = decltype(traj_execution_response_future_){};
                if (!traj_execution_goal_handle_) {
                    RCLCPP_ERROR(get_logger(), "Trajectory execution request was rejected, aborting MoveHandThroughPoses");
                    abortPoseIncrement();
                    return BT::NodeStatus::FAILURE;
                }
                return BT::NodeStatus::RUNNING;
            }
        }
    } else if (!traj_execution_goal_handle_){
        RCLCPP_ERROR(get_logger(), "MoveHandThroughPoses has no active action or action request. This should never happen");
        return BT::NodeStatus::FAILURE;
    }

    // Possibility two - goal is active and we check its status
    rclcpp::spin_some(this->get_node_base_interface());
    const int8_t goal_status = traj_execution_goal_handle_->get_status();
    switch (goal_status){
        case action_msgs::msg::GoalStatus::STATUS_CANCELING:
        case action_msgs::msg::GoalStatus::STATUS_ACCEPTED:
        case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
            return BT::NodeStatus::RUNNING;

        case action_msgs::msg::GoalStatus::STATUS_UNKNOWN:
            RCLCPP_WARN(get_logger(), "TrajectoryExecution action returned status UNKNOWN, reporting failure");
            [[fallthrough]];
        case action_msgs::msg::GoalStatus::STATUS_ABORTED:
        case action_msgs::msg::GoalStatus::STATUS_CANCELED:
            RCLCPP_WARN(get_logger(), "TrajectoryExecution action failed");
            traj_execution_goal_handle_.reset();
            return next_idx_ >= waypoints_.poses.size() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
            
        case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
            RCLCPP_INFO(get_logger(), "MoveHandThroughPoses: TrajectoryExecution action complete");
            traj_execution_goal_handle_.reset();
            return next_idx_ >= waypoints_.poses.size() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
    }

    RCLCPP_ERROR(get_logger(), "TrajectoryExecution action returned unknown status code \"%d\", reporting failure", +goal_status);
    return BT::NodeStatus::FAILURE;
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

void MoveHandThroughPoses::cancelOngoingTrajectoryExecutionRequest() {
    if (traj_execution_response_future_.valid()) {
        abortPoseIncrement();
        traj_execution_response_future_ = decltype(traj_execution_response_future_){};
    }
}
    
// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

void MoveHandThroughPoses::cancelOngoingTrajectoryExecution() {
    if (traj_execution_goal_handle_) {
        abortPoseIncrement();
        traj_execution_action_client_->async_cancel_goal(traj_execution_goal_handle_);
    }
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

bool MoveHandThroughPoses::hasOngoingMoveGroupRequest() const {
    return move_group_response_future_.valid() ||  move_group_goal_handle_ != nullptr;
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

BT::NodeStatus MoveHandThroughPoses::checkMoveGroupRequest() {
    // Check to see if we're still waiting on a response from the action server
    if (move_group_response_future_.valid()){
        const auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(), move_group_response_future_, std::chrono::milliseconds(5));
        switch (result){
            case rclcpp::FutureReturnCode::SUCCESS:
                move_group_goal_handle_ = move_group_response_future_.get();
                move_group_response_future_ = decltype(move_group_response_future_){};
                if (!move_group_goal_handle_) {
                    RCLCPP_ERROR(get_logger(), "Move group planning failed, aborting.");
                    return BT::NodeStatus::FAILURE;
                }
                return BT::NodeStatus::RUNNING;

            case rclcpp::FutureReturnCode::TIMEOUT:{
                const auto max_duration = std::chrono::milliseconds(static_cast<int>(1000*(max_planning_time_ + 5)));
                if (now() - move_group_request_timestamp_ > max_duration){
                    RCLCPP_ERROR(get_logger(), "Timed out waiting for MoveGroup action server to respond. Aborting MoveHandToPose behavior");
                    move_group_action_client_->async_cancel_all_goals();
                    return BT::NodeStatus::FAILURE;
                }
                else return BT::NodeStatus::RUNNING;
            }

            case rclcpp::FutureReturnCode::INTERRUPTED:
                move_group_response_future_ = decltype(move_group_response_future_){};
                RCLCPP_WARN(get_logger(), "MoveHandToPose MoveGroup request interrupted. Reporting failed movement");
                return BT::NodeStatus::FAILURE;
        }
    } else if (!move_group_goal_handle_) {
        RCLCPP_ERROR(get_logger(), "MoveHandToPose has no active action or action request. This should never happen");
        return BT::NodeStatus::FAILURE;
    }

    // Check if the action is ongoing, or if it has concluded
    rclcpp::spin_some(this->get_node_base_interface());
    const int8_t goal_status = move_group_goal_handle_->get_status();
    switch (goal_status){
        case action_msgs::msg::GoalStatus::STATUS_CANCELING:
        case action_msgs::msg::GoalStatus::STATUS_ACCEPTED:
        case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
            return BT::NodeStatus::RUNNING;

        case action_msgs::msg::GoalStatus::STATUS_UNKNOWN:
            RCLCPP_WARN(get_logger(), "MoveGroup action returned status UNKNOWN, reporting failure");
            [[fallthrough]];
        case action_msgs::msg::GoalStatus::STATUS_ABORTED:
        case action_msgs::msg::GoalStatus::STATUS_CANCELED:
            RCLCPP_WARN(get_logger(), "MoveGroup action failed");
            move_group_goal_handle_.reset();
            return ++next_idx_ >= waypoints_.poses.size() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;

        case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
            RCLCPP_INFO(get_logger(), "MoveHandToPose: MoveGroup Action complete");
            move_group_goal_handle_.reset();
            return ++next_idx_ >= waypoints_.poses.size() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
    }

    RCLCPP_ERROR(get_logger(), "MoveGroup action returned unknown status code \"%d\", reporting failure", +goal_status);
    return BT::NodeStatus::FAILURE;
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

void MoveHandThroughPoses::cancelOngoingMoveGroupRequest() {
    if (move_group_response_future_.valid()) {
        move_group_response_future_ = decltype(move_group_response_future_){};
    }
    if (move_group_goal_handle_) {
        move_group_action_client_->async_cancel_goal(move_group_goal_handle_);
    }
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

template<typename T> requires (!HasNewFeatures<T>)
void MoveHandThroughPoses::setIronVals(typename T::SharedPtr) {}

template<typename T> requires (HasNewFeatures<T>)
void MoveHandThroughPoses::setIronVals(typename T::SharedPtr req) {
    req->max_velocity_scaling_factor = 0.6*max_velocity_scaling_factor_;
    req->cartesian_speed_limited_link = getInput<std::string>("target_link").value_or("arm0_hand");
    req->max_cartesian_speed = max_end_effector_velocity_;
}

bool MoveHandThroughPoses::makeNewPathRequest() {
    auto req = std::make_shared<moveit_msgs::srv::GetCartesianPath::Request>();
    auto next_poses = waypoints_.poses | std::views::drop(next_idx_);
    
    req->header = waypoints_.header;
    req->group_name = planning_group_;
    req->link_name = getInput<std::string>("target_link").value_or("arm0_hand");
    req->waypoints = std::vector(next_poses.begin(), next_poses.end());
    req->max_step = 0.01;
    req->avoid_collisions = true;

    // Only set these parts if we're in a compatible ROS version
    setIronVals<moveit_msgs::srv::GetCartesianPath::Request>(req);
    
    path_computation_response_timestamp_ = now();
    path_computation_response_future_ = path_computation_client_->async_send_request(req);
    RCLCPP_INFO(get_logger(), "Making request to calculate cartesian path through %zd waypoints", req->waypoints.size());
    return true;
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

bool MoveHandThroughPoses::makeNewTrajectoryExecutionRequest(
    moveit_msgs::srv::GetCartesianPath::Response::SharedPtr path) 
{
    moveit_msgs::action::ExecuteTrajectory::Goal goal;
    goal.trajectory = path->solution; 

    rclcpp_action::Client<moveit_msgs::action::ExecuteTrajectory>::SendGoalOptions opts;
    traj_execution_response_future_ = traj_execution_action_client_->async_send_goal(goal);  
    traj_execution_request_timestamp_ = now();
    return true;
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

bool MoveHandThroughPoses::makeNewMoveGroupRequest() {
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.pose = waypoints_.poses.at(next_idx_);
    target_pose.header = waypoints_.header;

    moveit_msgs::action::MoveGroup::Goal move_group_goal;
    move_group_goal.planning_options.plan_only = false;
    move_group_goal.planning_options.replan = false;
    move_group_goal.request.allowed_planning_time = max_planning_time_;
    move_group_goal.request.max_velocity_scaling_factor = max_velocity_scaling_factor_;
    move_group_goal.request.goal_constraints.push_back(
        kinematic_constraints::constructGoalConstraints(
            getInput<std::string>("target_link").value_or("arm0_hand"), 
            target_pose, 
            getInput<double>("position_tolerance").value_or(0.001),
            getInput<double>("angular_tolerance").value_or(0.01)
        )
    );
    move_group_goal.request.group_name = "arm";
    move_group_goal.request.workspace_parameters.header.frame_id = "base_link";
    move_group_goal.request.workspace_parameters.header.stamp = now();
    move_group_goal.request.workspace_parameters.min_corner.x = -1e9;
    move_group_goal.request.workspace_parameters.min_corner.y = -1e9;
    move_group_goal.request.workspace_parameters.min_corner.z = -1e9;
    move_group_goal.request.workspace_parameters.max_corner.x = +1e9;
    move_group_goal.request.workspace_parameters.max_corner.y = +1e9;
    move_group_goal.request.workspace_parameters.max_corner.z = +1e9;

    // Request the motion
    RCLCPP_INFO(get_logger(), "Made request of pose index %zd", next_idx_);
    move_group_response_future_ = move_group_action_client_->async_send_goal(move_group_goal);
    move_group_request_timestamp_ = move_group_goal.request.workspace_parameters.header.stamp;
    return true;
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

void MoveHandThroughPoses::incrementPoseIndex(std::ptrdiff_t offset) {
    last_idx_ = next_idx_;
    next_idx_ += offset;
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

void MoveHandThroughPoses::abortPoseIncrement() {
    next_idx_ = last_idx_;
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

} // namespace spot_behaviors
