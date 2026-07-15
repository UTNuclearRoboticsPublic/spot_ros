////////////////////////////////////////////////////////////////////////////////////////////
//      Title     : check_battery.hpp
//      Project   : spot_ros
////////////////////////////////////////////////////////////////////////////////////////////

#pragma once
#include <thread>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>

#include "spot_msgs/msg/battery_state_array.hpp"
#include "spot_behaviors/node_behavior_base.hpp"

namespace spot_behaviors {

class CheckBattery : public BT::SyncActionNode, public NodeBehaviorBase {
public:
    CheckBattery(const std::string& name, const BT::NodeConfiguration& config, tf2_ros::Buffer::SharedPtr tf_buffer);

    static BT::PortsList providedPorts();

    // Returns SUCCESS if battery is over a given threshold, FAILURE otherwise
    BT::NodeStatus tick() override;

private:
    // Subscriber to data
    rclcpp::Subscription<spot_msgs::msg::BatteryStateArray>::SharedPtr battery_sub_;

    // How much battery percentage is left
    std::optional<float> battery_percentage_;

    // Record the battery state obtained from the message
    void batteryCallback(spot_msgs::msg::BatteryStateArray::UniquePtr msg);
};

} // namespace spot_behaviors
