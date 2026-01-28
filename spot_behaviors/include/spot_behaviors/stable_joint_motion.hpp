////////////////////////////////////////////////////////////////////////////////////////////
//      Title     : stable_joint_motion.hpp
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
#include <moveit_msgs/srv/get_motion_plan.hpp>
#include <moveit_msgs/srv/get_cartesian_path.hpp>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include "spot_behaviors/node_behavior_base.hpp"
#include "spot_msgs/action/arm_cartesian_command.hpp"

namespace spot_behaviors{

/**
 * This class helps to overcome a mixed issue with controlling Spot's arm autonomously.
 * The base Boston Dyanamics API does not expose any way to avoid collisions when
 * commanding the arm. This makes pure BD-server operation infeasible for autonomous
 * applications. MoveIt, on the other hand, provides many obstacle aware planners which
 * output collision free joint trajectories. Unfortunately, directly executing these
 * trajectories on Spot does not have the desired effect because the robot's whole body
 * controller leans the robot body to maintain balance, causing the actual end effecto
 * trajectory to deviate from the planned version. This behavior overcomes these issues
 * by querying a MoveIt interface to generate joint trajectories and then converting these
 * back into end-effector trajectories to be executed piecewise (but smoothly) by the
 * Boston Dyanmics Python API
 */
class StableJointMotion : public BT::StatefulActionNode, public NodeBehaviorBase {
public:
    StableJointMotion(const std::string& name, const BT::NodeConfiguration& config, tf2_ros::Buffer::SharedPtr tf_buffer);

    /** We accept x input ports - TODO 
     *  - target_pose: geometry_msgs::msg::PoseStamped 
     */
    static BT::PortsList providedPorts();

    /** 
     * Make the the motion request. 
     * @return RUNNING if the action server is available, FAILURE otherwise
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
    // Parameters (default values are provided at parameter declaration)
    double max_planning_time_{};
    std::string planning_group_{};
    double max_velocity_scaling_factor_{};
    double max_acceleration_scaling_factor_{};
    std::string target_link_;
    geometry_msgs::msg::PoseArray::SharedPtr waypoints_;

    // Robot model
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    moveit::core::RobotModelPtr robot_model_;
    moveit::core::RobotStatePtr robot_state_;

    // Service client
    rclcpp::Client<moveit_msgs::srv::GetMotionPlan>::SharedPtr moveit_plan_client_;
    std::optional<rclcpp::Client<moveit_msgs::srv::GetMotionPlan>::FutureAndRequestId> moveit_plan_future_;
    rclcpp::Time cartesian_request_timestamp_{};

    // Action client
    rclcpp_action::Client<spot_msgs::action::ArmCartesianCommand>::SharedPtr bosdyn_action_client_;

    // Action client future handle - only used while waiting for a request to be accepted or jejected
    std::shared_future<rclcpp_action::ClientGoalHandle<spot_msgs::action::ArmCartesianCommand>::SharedPtr> bosdyn_response_future_;
    rclcpp::Time request_timestamp_{};

    // Action client goal handle - nullptr if no request is active
    rclcpp_action::ClientGoalHandle<spot_msgs::action::ArmCartesianCommand>::SharedPtr bosdyn_goal_handle_;
    rclcpp::Time motion_start_time_{};

    // Check the status of an active motion goal request (NOTE: not the goal itself, but the request for the goal)
    BT::NodeStatus checkRequestStatus();

    // Check the status of an ongoing cartesian request
    BT::NodeStatus checkActiveMoveitPlanningRequest();

    // Check the status of an active motion
    BT::NodeStatus checkActiveGoalStatus();

    // Given a cartesian joint trajectory, generate a generic trajectory with both joint and end effector positions
    spot_msgs::action::ArmCartesianCommand::Goal::SharedPtr generateGenericTrajectory(moveit_msgs::srv::GetMotionPlan::Response::SharedPtr resp);
};

} // namespace spot_behaviors
