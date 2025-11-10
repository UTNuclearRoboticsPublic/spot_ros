#include <tf2_eigen/tf2_eigen.hpp>
#include "spot_behaviors/stable_joint_motion.hpp"

namespace spot_behaviors {

StableJointMotion::StableJointMotion(
    const std::string& name,
    const BT::NodeConfiguration& config,
    tf2_ros::Buffer::SharedPtr tf_buffer) : 
StatefulActionNode(name, config),
NodeBehaviorBase(name, tf_buffer)
{
    // TODO: Set all services and clients
}

BT::PortsList StableJointMotion::providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::PoseArray::SharedPtr>("waypoints", "The poses to move the arm through"),
        BT::InputPort<std::string>("target_link", "The link on the robot for which the path is to be executed"),
        BT::InputPort<double>("max_planning_time", 10.0, "The maximum allowable time before abandoning the planning request and returning FAILURE"),

        BT::OutputPort<geometry_msgs::msg::PoseArray::SharedPtr>("remaining_poses", "The poses not yet visited by the plan")
    };
}

BT::NodeStatus StableJointMotion::onStart() {
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
    auto cartesian_req = std::make_shared<moveit_msgs::srv::GetCartesianPath::Request>();
    cartesian_req->avoid_collisions = true;
    cartesian_req->group_name = "arm";
    cartesian_req->header = waypoints_->header;
    cartesian_req->waypoints = waypoints_->poses;
    cartesian_req->max_step = 0.05;
    cartesian_req->link_name = target_link_;

    cartesian_path_future_ = cartesian_path_client_->async_send_request(cartesian_req);
    cartesian_request_timestamp_ = now();
    return BT::NodeStatus::SUCCESS;
}
    
BT::NodeStatus StableJointMotion::onRunning() {
    if (cartesian_path_future_) {
        return checkActiveCartesianRequest();
    }

    // TODO: Create a goal for the StableJointMotion action request

    // TODO: Check the state of the running StableJointMotion action request

    // None of the expected cases were true - return failure
    RCLCPP_ERROR(get_logger(), "Behavior is in an unspecified state! Returning failure");
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus StableJointMotion::checkActiveCartesianRequest() {
    auto status = rclcpp::spin_until_future_complete(this->get_node_base_interface(), cartesian_path_future_->future, std::chrono::milliseconds(5));
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
            moveit_msgs::srv::GetCartesianPath::Response::SharedPtr resp = cartesian_path_future_->get();
            cartesian_path_future_.reset();
            if (resp->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
                RCLCPP_ERROR(get_logger(), "Unable to find a cartesian path through the poses");
                return BT::NodeStatus::FAILURE;
            }

            const int num_waypoints_achieved = static_cast<int>(resp->fraction*waypoints_->poses.size());
            if (num_waypoints_achieved == 0) {
                RCLCPP_WARN(get_logger(), "Unable to reach the first waypoint. Reporting FAILURE");
                return BT::NodeStatus::FAILURE;
            } else {
                RCLCPP_INFO(get_logger(), "Found a carteisan path for %d (%.2f%%) of the waypoints", num_waypoints_achieved, 100.0*resp->fraction);
                moveit_msgs::msg::GenericTrajectory::SharedPtr generic_trajectory = generateGenericTrajectory(resp);
                geometry_msgs::msg::PoseArray::SharedPtr remaining_poses = std::make_shared<geometry_msgs::msg::PoseArray>();
                remaining_poses->header = waypoints_->header;
                std::ranges::copy(waypoints_->poses | std::views::drop(num_waypoints_achieved), std::back_inserter(remaining_poses->poses));
                setOutput("remaining_poses", remaining_poses);
                return BT::NodeStatus::RUNNING;
            }
        }
    }
}

moveit_msgs::msg::GenericTrajectory::SharedPtr StableJointMotion::generateGenericTrajectory(moveit_msgs::srv::GetCartesianPath::Response::SharedPtr resp) {
    auto generic_trajectory = std::make_shared<moveit_msgs::msg::GenericTrajectory>();
    generic_trajectory->joint_trajectory.push_back(resp->solution.joint_trajectory);
    moveit_msgs::msg::CartesianTrajectory& cartesian_trajectory = generic_trajectory->cartesian_trajectory.emplace_back();
    for (const trajectory_msgs::msg::JointTrajectoryPoint& joint_pos : resp->solution.joint_trajectory.points) {
        // Update the arm state
        moveit::core::JointModelGroup* arm_group = robot_model_->getJointModelGroup("arm");
        robot_state_->setJointGroupPositions(arm_group, joint_pos.positions);
        robot_state_->setJointGroupVelocities(arm_group, joint_pos.velocities);
        robot_state_->setJointGroupAccelerations(arm_group, joint_pos.accelerations);
        robot_state_->updateLinkTransforms();

        // Get the end effector position and set it in the trajectory
        const Eigen::Isometry3d ee_pose = robot_state_->getGlobalLinkTransform("arm0_hand");
        cartesian_trajectory.tracked_frame = "arm0_hand";
        moveit_msgs::msg::CartesianTrajectoryPoint& traj_point = cartesian_trajectory.points.emplace_back();
        traj_point.time_from_start = joint_pos.time_from_start;
        traj_point.point.pose = tf2::toMsg(ee_pose);

        // Get the end effector velocity and set it in the trajectory
        Eigen::MatrixXd jacobian = robot_state_->getJacobian(arm_group);
        Eigen::VectorXd joint_velocities;
        robot_state_->copyJointGroupVelocities(arm_group, joint_velocities);
        auto ee_vel = jacobian * joint_velocities;
        traj_point.point.velocity.linear.x = ee_vel[0];
        traj_point.point.velocity.linear.y = ee_vel[1];
        traj_point.point.velocity.linear.z = ee_vel[2];
        traj_point.point.velocity.angular.x = ee_vel[3];
        traj_point.point.velocity.angular.y = ee_vel[4];
        traj_point.point.velocity.angular.z = ee_vel[5];
    }

    return generic_trajectory;
}

void StableJointMotion::onHalted() {
    if (cartesian_path_future_) {
        cartesian_path_client_->remove_pending_request(*cartesian_path_future_);
        cartesian_path_future_.reset();
    }
}

} // namespace spot_behaviors
