////////////////////////////////////////////////////////////////////////////////////////////
//      Title     : tigger_service.hpp
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
#include <std_srvs/srv/trigger.hpp>
#include "behaviortree_cpp_v3/action_node.h"

namespace spot_behaviors {

class TriggerService : public BT::StatefulActionNode {
public:
    TriggerService(const std::string& name, const BT::NodeConfiguration& config);

    /** 
     * We accept 1 input port: service name 
     */
    BT::PortsList providedPorts();

    /** 
     * Make the the trigger request. 
     * @return RUNNING if the service is available, FAILURE otherwise
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

private:
    // The node instance
    rclcpp::Node::SharedPtr node_;

    // Used to prevent node namespace clashes
    static inline int node_count_ = 0;

    // Service client
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr trigger_client_;

    // Serivce client future result - empty optional if no request is active
    std::optional<rclcpp::Client<std_srvs::srv::Trigger>::FutureAndRequestId> service_future_;
};

} // namespace spot_behaviors
