////////////////////////////////////////////////////////////////////////////////////////////
//      Title     : check_arm_stowed.cpp
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

#include "spot_behaviors/check_arm_stowed.hpp"

namespace spot_behaviors {

CheckArmStowed::CheckArmStowed(const std::string& name, const BT::NodeConfiguration& config, tf2_ros::Buffer::SharedPtr tf_buffer) :
    BT::SyncActionNode(name, config),
    NodeBehaviorBase(name, tf_buffer)
    {
        manipulator_sub_ = this->create_subscription<spot_msgs::msg::ManipulatorStowState>(
            "/spot_manipulation_driver/manipulator_state/stow_state",
            rclcpp::ParametersQoS{},
            std::bind(&CheckArmStowed::manipulatorStateCallback, this, std::placeholders::_1)
        );
        spin_thread_ = std::thread([this](){rclcpp::spin(this->get_node_base_interface());});
    }

BT::NodeStatus CheckArmStowed::tick() {
    // Wait a little for messages to come through
    rclcpp::sleep_for(std::chrono::milliseconds(1000));

    if (!arm_is_stowed_.has_value()){
        RCLCPP_ERROR(get_logger(), "No messages received on topic %s", manipulator_sub_->get_topic_name());
        return BT::NodeStatus::FAILURE;
    }

    const bool arm_is_stowed = arm_is_stowed_.value();
    arm_is_stowed_ = std::nullopt;
    RCLCPP_INFO(get_logger(), "Arm is %sstowed", arm_is_stowed ? "" : "un");
    return arm_is_stowed ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void CheckArmStowed::manipulatorStateCallback(spot_msgs::msg::ManipulatorStowState::UniquePtr msg){
    arm_is_stowed_ = (msg->state == msg->STOWSTATE_STOWED);
}

} // namespace spot_behaviors
