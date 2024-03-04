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
