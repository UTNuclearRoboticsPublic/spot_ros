////////////////////////////////////////////////////////////////////////////////////////////
//      Title     : set_simulated_pose.hpp
//      Project   : spot_ros
//      Copyright : Copyright© The University of Texas at Austin, 2026. All rights reserved.
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
#include <thread>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>

#include "spot_behaviors/node_behavior_base.hpp"
#include "spot_msgs/srv/set_simulated_pose.hpp"

namespace spot_behaviors {

class SetSimulatedPose : public BT::StatefulActionNode, public NodeBehaviorBase {
public:
    SetSimulatedPose(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();

    // Returns SUCCESS if the service returns successfully, FAILURE otherwise
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    rclcpp::Client<spot_msgs::srv::SetSimulatedPose>::SharedPtr pose_set_client_;
    spot_msgs::srv::SetSimulatedPose::Response::SharedPtr pose_set_response_;
    rclcpp::Time query_start_time_;
};

} // namespace spot_behaviors
