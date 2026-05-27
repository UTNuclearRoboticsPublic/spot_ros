////////////////////////////////////////////////////////////////////////////////////////////
//      Title     : set_simulated_pose.hpp
//      Project   : spot_ros
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
