#pragma once
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <spot_msgs/msg/gesture.hpp>
#include <spot_msgs/srv/gesture_sequence.hpp>
#include "behaviortree_cpp/action_node.h"
#include "spot_behaviors/node_behavior_base.hpp"

namespace spot_behaviors {

class GestureSequence : public BT::StatefulActionNode, public NodeBehaviorBase {
public:
    GestureSequence(const std::string& name, const BT::NodeConfiguration& config, tf2_ros::Buffer::SharedPtr tf_buffer = nullptr);

    /** We accept 2 input port - gesture_sequence and gesture_mode 
     *  Declares an input port named "gesture_sequence" that accepts a vector of Gesture messages.
     *  This input is expected to be a sequence of gestures (custom messages of type spot_msgs::msg::Gesture),
     *  Each gesture is composed of 6 float32 types representing roll, pitch, yaw, body_height, and pose_duration.
     *  roll: (-0.6, 0.6)
     *  pitch: (-0.6, 0.6)
     *  yaw: (-0.6, 0.6)
     *  body_height: (-0.2, 0.2)
     *  pose_duration: [0, infinity]
    */
    static BT::PortsList providedPorts();

    /** 
     * Make the the gesture sequence request. 
     * @return RUNNING if the service is available, FAILURE otherwise
     */
    BT::NodeStatus onStart() override;

    /**
     * Check the status of an existing request
     * @return RUNNING if request is not yet complete, FAILURE if no requset has
     *         been made, SUCCESS/FAILURE if the request was completed  
     */
    BT::NodeStatus onRunning() override;

    /**
     * If an there is an existing request, cancel it 
     */
    void onHalted() override;

private:
    // Service client
    rclcpp::Client<spot_msgs::srv::GestureSequence>::SharedPtr gesture_sequence_client_;
    rclcpp::Time request_timestamp_;

    // Request variable containing service field info
    std::shared_ptr<spot_msgs::srv::GestureSequence::Request> request_;

    // Total duration of the gesture sequence (seconds)
    float sequence_duration_;

    // Serivce client future result - empty optional if no request is active
    std::optional<rclcpp::Client<spot_msgs::srv::GestureSequence>::FutureAndRequestId> service_future_;
};

} // namespace spot_behaviors
