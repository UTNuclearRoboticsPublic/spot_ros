#include <geometry_msgs/msg/pose_stamped.hpp>
#include "spot_behaviors/record_current_location.hpp"

namespace spot_behaviors{
    
RecordCurrentLocation::RecordCurrentLocation(const std::string& name, const BT::NodeConfiguration& config) :
    BT::SyncActionNode(name, config),
    node_(std::make_shared<rclcpp::Node>(name+"BT"+std::to_string(node_count_++), "spot_behaviors")),
    tf_buffer_(node_->get_clock()),
    tf_listener_(tf_buffer_)
{}

BT::PortsList RecordCurrentLocation::providedPorts() {
    return {
        BT::InputPort<std::string>("global_frame", "The frame in which to locate the robot. Defaults to 'map'"),
        BT::InputPort<std::string>("robot_frame", "The frame whose location to record. Defaults to 'base_link'"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("recorded_pose", "The measured pose")
    };
}

BT::NodeStatus RecordCurrentLocation::tick() {
    const std::string global_frame = getInput<std::string>("global_frame").value_or("map");
    const std::string query_frame = getInput<std::string>("robot_frame").value_or("base_link");

    if (std::string err; !tf_buffer_.canTransform(global_frame, query_frame, tf2::TimePointZero, std::chrono::seconds(2), &err)) {
        RCLCPP_ERROR(node_->get_logger(), "Unable to record frame location: %s", err.c_str());
        return BT::NodeStatus::FAILURE;
    }

    const geometry_msgs::msg::TransformStamped frame_transform = tf_buffer_.lookupTransform(
        global_frame,
        query_frame,
        tf2::TimePointZero
    );

    geometry_msgs::msg::PoseStamped frame_pose;
    frame_pose.header.stamp = rclcpp::Time(0);
    frame_pose.header.frame_id = global_frame;
    frame_pose.pose.position.x = frame_transform.transform.translation.x;
    frame_pose.pose.position.y = frame_transform.transform.translation.y;
    frame_pose.pose.position.z = frame_transform.transform.translation.z;
    frame_pose.pose.orientation = frame_transform.transform.rotation;

    setOutput<geometry_msgs::msg::PoseStamped>("recorded_pose", frame_pose);
    return BT::NodeStatus::SUCCESS;
}

} // namespace spot_behaviors
