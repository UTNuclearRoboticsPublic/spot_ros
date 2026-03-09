////////////////////////////////////////////////////////////////////////////////////////////
//      Title     : check_battery.cpp
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

#include "spot_behaviors/check_battery.hpp"

namespace spot_behaviors {

CheckBattery::CheckBattery(const std::string& name, const BT::NodeConfiguration& config, tf2_ros::Buffer::SharedPtr tf_buffer) :
    BT::SyncActionNode(name, config),
    NodeBehaviorBase(name, tf_buffer)
    {}

BT::PortsList CheckBattery::providedPorts() {
    return {
        BT::InputPort<float>("battery_threshold"),
        BT::InputPort<float>("timeout")
    };
}

BT::NodeStatus CheckBattery::tick() {
    // Start up the subscription
    battery_sub_ = this->create_subscription<spot_msgs::msg::BatteryStateArray>(
        "/spot_driver/status/battery_states",
        rclcpp::SensorDataQoS{},
        std::bind(&CheckBattery::batteryCallback, this, std::placeholders::_1)
    );

    // Wait a little for messages to come through
    const float timeout_seconds = getInput<float>("timeout").value_or(2.0);
    auto elapsed_time = [start_time = std::chrono::steady_clock::now()]() {
        return std::chrono::duration_cast<std::chrono::duration<float>>(std::chrono::steady_clock::now() - start_time).count();
    };
    while (!battery_percentage_.has_value() && (elapsed_time() < timeout_seconds)) {
        rclcpp::spin_some(get_node_base_interface());
    }

    if (!battery_percentage_.has_value()){
        RCLCPP_ERROR(get_logger(), "No messages received on topic %s", battery_sub_->get_topic_name());
        return BT::NodeStatus::FAILURE;
    }

    BT::Expected<float> battery_threshold = getInput<float>("battery_threshold");
    if (!battery_threshold.has_value()){
        RCLCPP_ERROR(get_logger(), "No battery threshold provided for BT Node %s", this->name().c_str());
        return BT::NodeStatus::FAILURE;
    }

    const float battery_percentage = battery_percentage_.value();
    battery_percentage_ = std::nullopt;
    if (battery_percentage >= battery_threshold.value()) {
        return BT::NodeStatus::SUCCESS;
    } else {
        RCLCPP_ERROR(get_logger(), "Robot battery (%.0f%%) is below threshold value of %.0f%%, aborting behavior tree execution", battery_percentage, battery_threshold.value());
        return BT::NodeStatus::FAILURE;
    }
}

void CheckBattery::batteryCallback(spot_msgs::msg::BatteryStateArray::UniquePtr msg){
    battery_percentage_ = msg->battery_states.at(0).charge_percentage;
    battery_sub_.reset();
}

} // namespace spot_behaviors
