#pragma once
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/action_node.h>

#include "spot_msgs/msg/battery_state.hpp"

namespace spot_behaviors {

class CheckBattery : public BT::SyncActionNode {
public:
    CheckBattery(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();

    // Returns SUCCESS if battery is over a given threshold, FAILURE otherwise
    BT::NodeStatus tick() override;

private:
    // The node instance to use to collect data
    rclcpp::Node::SharedPtr node_;

    // Subscriber to data
    rclcpp::Subscription<spot_msgs::msg::BatteryState>::SharedPtr battery_sub_;

    // How much battery percentage is left
    std::optional<float> battery_percentage_;

    // How many instances of such BT nodes have been created (used to avoid namespace conflicts)
    static inline int node_count_ = 0;

    // Record the battery state obtained from the message
    void batteryCallback(spot_msgs::msg::BatteryState::UniquePtr msg);
};

} // namespace spot_behaviors
