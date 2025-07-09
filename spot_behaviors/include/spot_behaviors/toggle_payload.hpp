////////////////////////////////////////////////////////////////////////////////////////////
//      Title     : toggle_payload.hpp
//      Project   : spot_ros
//      Copyright : Copyright© The University of Texas at Austin, 2025. All rights reserved.
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

#include "spot_msgs/srv/toggle_payload.hpp"
#include "spot_behaviors/node_behavior_base.hpp"

namespace spot_behaviors {

class TogglePayload : public BT::StatefulActionNode, public NodeBehaviorBase {
public:
    TogglePayload(const std::string& name, const BT::NodeConfig& config, tf2_ros::Buffer::SharedPtr tf_buffer);

    /**
     * We accept 3 inputs ports:
     *   - guid: The GUID of the payload to toggle
     *   - name: The name of the payload to toggle
     *   - attached: Whether the payload should be attached or not after this operation
     */
    static BT::PortsList providedPorts();

    /**
     * Make the request
     * @return RUNNING if inputs are properly configured, FAILURE otherwise
     */
    BT::NodeStatus onStart() override;

    /**
     * Check to see if the server has responded
     * @return RUNNING if the request is not yet complete,
     *         SUCCESS/FAILURE if the request was completed
     */
    BT::NodeStatus onRunning() override;

    /**
     * Clean up request values
     */
    void onHalted() override;
private:
    rclcpp::Time request_start_time_;
    rclcpp::Client<spot_msgs::srv::TogglePayload>::SharedPtr payload_client_;
    std::optional<rclcpp::Client<spot_msgs::srv::TogglePayload>::FutureAndRequestId> service_future_;
    float timeout_;
};

} // namespace spot_behaviors
