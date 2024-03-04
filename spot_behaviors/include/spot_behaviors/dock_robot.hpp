#pragma once
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <spot_msgs/srv/dock.hpp>
#include "behaviortree_cpp_v3/action_node.h"

namespace spot_behaviors {

class DockRobot : public BT::StatefulActionNode {
public:
    DockRobot(const std::string& name, const BT::NodeConfiguration& config);

    /** We accept 1 input port - dock_id */
    BT::PortsList providedPorts();

    /** 
     * Make the the dock request. 
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
    rclcpp::Client<spot_msgs::srv::Dock>::SharedPtr dock_client_;

    // Serivce client future result - empty optional if no request is active
    std::optional<rclcpp::Client<spot_msgs::srv::Dock>::FutureAndRequestId> service_future_;
};

} // namespace spot_behaviors
