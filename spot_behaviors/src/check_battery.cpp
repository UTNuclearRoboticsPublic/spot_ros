#include "spot_behaviors/check_battery.hpp"

namespace spot_behaviors {

CheckBattery::CheckBattery(const std::string& name, const BT::NodeConfiguration& config) :
    BT::SyncActionNode(name, config),
    node_(std::make_shared<rclcpp::Node>(name+"BT"+std::to_string(node_count_++)))
    {
        battery_sub_ = node_->create_subscription<spot_msgs::msg::BatteryState>(
            "/spot_driver/status/battery_state",
            rclcpp::ParametersQoS{},
            std::bind(&CheckBattery::batteryCallback, this, std::placeholders::_1)
        );
    }

BT::PortsList CheckBattery::providedPorts() {
    return {
        BT::InputPort<float>("battery_threshold")
    };
}

BT::NodeStatus CheckBattery::tick() {
    if(!battery_percentage_.has_value()){
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    rclcpp::spin_some(node_);

    if (!battery_percentage_.has_value()){
        RCLCPP_WARN(node_->get_logger(), "No messages received on topic %s", battery_sub_->get_topic_name());
        return BT::NodeStatus::FAILURE;
    }

    BT::Optional<float> battery_threshold = getInput<float>("battery_threshold");
    if (!battery_threshold.has_value()){
        RCLCPP_WARN(node_->get_logger(), "No battery threshold provided for BT Node %s", this->name().c_str());
        return BT::NodeStatus::FAILURE;
    }

    return battery_percentage_.value() >= battery_threshold.value() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void CheckBattery::batteryCallback(spot_msgs::msg::BatteryState::UniquePtr msg){
    battery_percentage_ = msg->charge_percentage;
}

} // namespace spot_behaviors
