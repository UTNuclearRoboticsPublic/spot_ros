////////////////////////////////////////////////////////////////////////////////////////////
//      Title     : check_arm_stowed.cpp
//      Project   : spot_ros
////////////////////////////////////////////////////////////////////////////////////////////

#include "spot_behaviors/check_arm_stowed.hpp"

namespace spot_behaviors {

CheckArmStowed::CheckArmStowed(const std::string& name, const BT::NodeConfiguration& config, tf2_ros::Buffer::SharedPtr tf_buffer) :
    BT::SyncActionNode(name, config),
    NodeBehaviorBase(name, tf_buffer)
    {}

BT::NodeStatus CheckArmStowed::tick() {
    // Create the subscriber
    manipulator_sub_ = this->create_subscription<spot_msgs::msg::ManipulatorStowState>(
        "/spot_manipulation_driver/manipulator_state/stow_state",
        rclcpp::ParametersQoS{},
        std::bind(&CheckArmStowed::manipulatorStateCallback, this, std::placeholders::_1)
    );

    // Wait a little for messages to come through
    const float timeout_seconds = 1.0f;
    auto elapsed_time = [start_time = std::chrono::steady_clock::now()]() {
        return std::chrono::duration_cast<std::chrono::duration<float>>(std::chrono::steady_clock::now() - start_time).count();
    };

    while (!arm_is_stowed_.has_value() && (elapsed_time() < timeout_seconds)) {
        rclcpp::spin_some(get_node_base_interface());
    }

    if (!arm_is_stowed_.has_value()){
        RCLCPP_ERROR(get_logger(), "No messages received on topic %s", manipulator_sub_->get_topic_name());
        return BT::NodeStatus::FAILURE;
    }

    const bool arm_is_stowed = arm_is_stowed_.value();
    arm_is_stowed_.reset();
    RCLCPP_INFO(get_logger(), "Arm is %sstowed", arm_is_stowed ? "" : "un");
    return arm_is_stowed ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void CheckArmStowed::manipulatorStateCallback(spot_msgs::msg::ManipulatorStowState::UniquePtr msg){
    arm_is_stowed_ = (msg->state == msg->STOWSTATE_STOWED);
    manipulator_sub_.reset();
}

} // namespace spot_behaviors
