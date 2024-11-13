////////////////////////////////////////////////////////////////////////////////////////////
//      Title     : move_arm_through_poses.hpp
//      Project   : spot_ros
//      Copyright : Copyright© The University of Texas at Austin, 2024. All rights reserved.
//                
//          All files within this directory are subject to the following, unless an alternative
//          license is explicitly included within the text of each file.
//
//          This software and documentation constitute an unpublished work
//          and contain valuable trade secrets and proprietary information
//          belonging to the University. None of the foregoing material may be
//          copied or duplicated or disclosed without the express, written
//          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
//          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
//          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
//          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
//          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
//          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
//          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
//          University be liable for incidental, special, indirect, direct or
//          consequential damages or loss of profits, interruption of business,
//          or related expenses which may arise from use of software or documentation,
//          including but not limited to those resulting from defects in software
//          and/or documentation, or loss or inaccuracy of data of any kind.
//
////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/srv/get_cartesian_path.hpp>
#include <moveit_msgs/action/execute_trajectory.hpp>

#include "spot_behaviors/node_behavior_base.hpp"

namespace spot_behaviors{

class MoveHandThroughPoses : public BT::StatefulActionNode, public NodeBehaviorBase {
public:
    MoveHandThroughPoses(const std::string& name, const BT::NodeConfiguration& config, tf2_ros::Buffer::SharedPtr tf_buffer);

    /** We accept 1 input ports 
     *  - waypoints: geometry_msgs::msg::PoseArray 
     */
    static BT::PortsList providedPorts();

    /** 
     * Verify servers are present and inputs are valid
     * @return RUNNING if well configured, FAILURE otherwise
     */
    BT::NodeStatus onStart() override;

    /**
     * Check the status of an existing request
     * @return RUNNING if request is not yet complete, FAILURE if no requset has
     *         been made, SUCCESS/FAILURE if the request was completed  
     */
    BT::NodeStatus onRunning() override;

    /**
     * If an there is an existing request, cancel it 
     */
    void onHalted() override;

protected:
    // The index of the next pose to start from
    std::size_t last_idx_ = 0;
    std::size_t next_idx_ = 0;

    // Parameters (default values are provided at parameter declaration)
    double max_planning_time_{};
    double max_cartesian_planning_time_{};
    std::string planning_group_{};
    double max_velocity_scaling_factor_{};
    double max_end_effector_velocity_{};

    // Storage for the waypoints that we receive from blackboard
    geometry_msgs::msg::PoseArray waypoints_;

    // Path computation client
    rclcpp::Client<moveit_msgs::srv::GetCartesianPath>::SharedPtr path_computation_client_;

    // Service client future handle
    std::optional<rclcpp::Client<moveit_msgs::srv::GetCartesianPath>::FutureAndRequestId> path_computation_response_future_;
    rclcpp::Time path_computation_response_timestamp_;

    // Action client
    rclcpp_action::Client<moveit_msgs::action::ExecuteTrajectory>::SharedPtr traj_execution_action_client_;

    // Action client future handle - only used while waiting for a request to be accepted or rejected
    std::shared_future<rclcpp_action::ClientGoalHandle<moveit_msgs::action::ExecuteTrajectory>::SharedPtr> traj_execution_response_future_;
    rclcpp::Time traj_execution_request_timestamp_{};

    // Action client goal handle - nullptr if no request is active
    rclcpp_action::ClientGoalHandle<moveit_msgs::action::ExecuteTrajectory>::SharedPtr traj_execution_goal_handle_;

    // MoveGroup action client - for non-Cartesian moves
    rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SharedPtr move_group_action_client_;
    
    std::shared_future<rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::SharedPtr> move_group_response_future_;
    rclcpp::Time move_group_request_timestamp_{};

    rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::SharedPtr move_group_goal_handle_;

    // Cartesian Path status update functions
    bool hasOngoingPathRequest() const;
    BT::NodeStatus checkPathRequestStatus();
    void cancelOngoingPathRequest();

    // Move group request status update functions
    bool hasOngoingMoveGroupRequest() const;
    BT::NodeStatus checkMoveGroupRequest();
    void cancelOngoingMoveGroupRequest();

    // Trajectory Execution status update functions 
    bool hasOngoingTrajectoryExecutionRequest() const;
    BT::NodeStatus checkTrajectoryExecutionStatus();
    void cancelOngoingTrajectoryExecutionRequest();
    void cancelOngoingTrajectoryExecution();

    // New request functions
    bool makeNewPathRequest();
    bool makeNewTrajectoryExecutionRequest(moveit_msgs::srv::GetCartesianPath::Response::SharedPtr path);
    bool makeNewMoveGroupRequest();

    // Functions for managing the current pose index
    void incrementPoseIndex(std::ptrdiff_t offset);
    void abortPoseIncrement();
};

} // namespace spot_behaviors
