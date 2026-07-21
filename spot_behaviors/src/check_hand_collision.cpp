////////////////////////////////////////////////////////////////////////////////////////////
//      Title     : check_hand_collision.cpp
//      Project   : spot_ros
////////////////////////////////////////////////////////////////////////////////////////////

#include "spot_behaviors/check_hand_collision.hpp"

namespace spot_behaviors {

CheckHandCollision::CheckHandCollision(const std::string& name, const BT::NodeConfiguration& config, tf2_ros::Buffer::SharedPtr tf_buffer) :
    BT::SyncActionNode(name, config),
    NodeBehaviorBase(name, tf_buffer)
    {
        manipulator_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/spot_manipulation_driver/manipulator_state/is_hand_in_collision",
            rclcpp::ParametersQoS{},
            std::bind(&CheckHandCollision::collisionStateCallback, this, std::placeholders::_1)
        );
    }

BT::PortsList CheckHandCollision::providedPorts() {
    return {};
}

BT::NodeStatus CheckHandCollision::tick() {
    rclcpp::spin_some(get_node_base_interface());

    if (!in_collision_.has_value()){
        RCLCPP_ERROR(get_logger(), "No messages received on topic %s", manipulator_sub_->get_topic_name());
        return BT::NodeStatus::FAILURE;
    }

    if (in_collision_.value()) {
        RCLCPP_WARN(get_logger(), "Hand is in collision");
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
}

void CheckHandCollision::collisionStateCallback(std_msgs::msg::Bool::UniquePtr msg){
    in_collision_ = msg->data;
}

} // namespace spot_behaviors
