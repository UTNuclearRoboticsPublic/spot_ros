////////////////////////////////////////////////////////////////////////////////////////////
//      Title     : execute_stable_arm_command.hpp
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
#include "spot_behaviors/node_behavior_base.hpp"
#include "spot_msgs/action/stable_arm_command.hpp"

namespace spot_behaviors{

/**
 * === ExecuteStableArmCommand ===

 * This behavior bypasses the motion planning step for arm motions using the
 * stable arm motion server by assuming that you already have a traejctory and
 * simply wish to execute it. This can happen when motion planning is performed 
 * by a dedicated node ahead of time, and when the state of the robot during
 * planning is not the current state of the robot. In that case, simply calling
 * the ExecuteTrajectory service of the stable motion server would calculate the 
 * end-effector poses relative to the current body pose, which would lead to minor
 * inaccuracies in position if the robot is currntly leaning due to a deployed arm.
 */

class ExecuteStableArmCommand : public BT::StatefulActionNode, public NodeBehaviorBase {
public:
    ExecuteStableArmCommand(const std::string& name, const BT::NodeConfiguration& config, tf2_ros::Buffer::SharedPtr tf_buffer);

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

    // Action client
    rclcpp_action::Client<spot_msgs::action::StableArmCommand>::SharedPtr motion_action_client_;

    // Action client future handle - only used while waiting for a request to be accepted or jejected
    std::optional<std::shared_future<rclcpp_action::ClientGoalHandle<spot_msgs::action::StableArmCommand>::SharedPtr>> motion_response_future_;
    rclcpp::Time request_timestamp_{};
    bool hasOngoingMotionRequest() const {return motion_response_future_.has_value();};
    BT::NodeStatus checkMotionRequestStatus();

    // Action client goal handle - nullptr if no request is active
    std::optional<bool> success_;
    rclcpp_action::ClientGoalHandle<spot_msgs::action::StableArmCommand>::SharedPtr motion_goal_handle_;
    std::optional<std::shared_future<rclcpp_action::ClientGoalHandle<spot_msgs::action::StableArmCommand>::WrappedResult>> motion_goal_future_;
    rclcpp::Time motion_start_time_;
    bool hasOngoingMotionExecution() const {return static_cast<bool>(motion_goal_handle_);}
    BT::NodeStatus checkMotionExecutionStatus();

    template<typename ActionType>
    BT::NodeStatus checkRequestStatus(std::shared_future<typename rclcpp_action::ClientGoalHandle<ActionType>::SharedPtr> shared_future);
};

} // namespace spot_behaviors
