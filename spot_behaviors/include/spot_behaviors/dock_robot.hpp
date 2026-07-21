////////////////////////////////////////////////////////////////////////////////////////////
//      Title     : dock_robot.hpp
//      Project   : spot_ros
////////////////////////////////////////////////////////////////////////////////////////////

#pragma once
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <spot_msgs/srv/dock.hpp>
#include "behaviortree_cpp/action_node.h"
#include "spot_behaviors/node_behavior_base.hpp"

namespace spot_behaviors {

class DockRobot : public BT::StatefulActionNode, public NodeBehaviorBase {
public:
    DockRobot(const std::string& name, const BT::NodeConfiguration& config, tf2_ros::Buffer::SharedPtr tf_buffer = nullptr);

    /** We accept 1 input port - dock_id */
    static BT::PortsList providedPorts();

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
    // Service client
    rclcpp::Client<spot_msgs::srv::Dock>::SharedPtr dock_client_;
    rclcpp::Time request_timestamp_;

    // Serivce client future result - empty optional if no request is active
    std::optional<rclcpp::Client<spot_msgs::srv::Dock>::FutureAndRequestId> service_future_;
};

} // namespace spot_behaviors
