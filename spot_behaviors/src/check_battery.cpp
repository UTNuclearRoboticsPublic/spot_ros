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

CheckBattery::CheckBattery(const std::string& name, const BT::NodeConfiguration& config) :
    BT::SyncActionNode(name, config),
    node_(std::make_shared<rclcpp::Node>(name+"BT"+std::to_string(node_count_++), "spot_behaviors"))
    {
        battery_sub_ = node_->create_subscription<spot_msgs::msg::BatteryStateArray>(
            "/spot_driver/status/battery_states",
            rclcpp::ParametersQoS{},
            std::bind(&CheckBattery::batteryCallback, this, std::placeholders::_1)
        );
        spin_thread_ = std::thread([this](){rclcpp::spin(node_);});
    }

BT::PortsList CheckBattery::providedPorts() {
    return {
        BT::InputPort<float>("battery_threshold")
    };
}

BT::NodeStatus CheckBattery::tick() {
    // Wait a little for messages to come through
    rclcpp::sleep_for(std::chrono::milliseconds(1000));

    if (!battery_percentage_.has_value()){
        RCLCPP_ERROR(node_->get_logger(), "No messages received on topic %s", battery_sub_->get_topic_name());
        return BT::NodeStatus::FAILURE;
    }

    BT::Expected<float> battery_threshold = getInput<float>("battery_threshold");
    if (!battery_threshold.has_value()){
        RCLCPP_ERROR(node_->get_logger(), "No battery threshold provided for BT Node %s", this->name().c_str());
        return BT::NodeStatus::FAILURE;
    }

    const float battery_percentage = battery_percentage_.value();
    battery_percentage_ = std::nullopt;
    return battery_percentage >= battery_threshold.value() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void CheckBattery::batteryCallback(spot_msgs::msg::BatteryStateArray::UniquePtr msg){
    battery_percentage_ = msg->battery_states.at(0).charge_percentage;
}

} // namespace spot_behaviors
