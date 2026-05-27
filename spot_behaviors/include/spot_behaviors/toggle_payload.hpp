////////////////////////////////////////////////////////////////////////////////////////////
//      Title     : toggle_payload.hpp
//      Project   : spot_ros
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
     * Inputs ports:
     *   - payload_guid: The GUID of the payload to toggle
     *   - payload_name: The name of the payload to toggle
     *   - secret: The value of the authentication secret, or path to a file that contains it
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
