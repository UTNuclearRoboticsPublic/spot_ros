////////////////////////////////////////////////////////////////////////////////////////////
//      Title     : get_spot_ik.hpp
//      Project   : spot_behaviors
////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <behaviortree_cpp/action_node.h>
#include <spot_msgs/srv/inverse_kinematics.hpp>
#include <spot_behaviors/node_behavior_base.hpp>

namespace spot_behaviors{

class GetSpotIK : public BT::StatefulActionNode, public NodeBehaviorBase {
public:
    GetSpotIK(const std::string& name, const BT::NodeConfiguration& config, tf2_ros::Buffer::SharedPtr tf_buffer = nullptr);

    /** We accept 3 input ports 
     *  - goal_pose: geometry_msgs::msg::PoseStamped::SharedPtr
     *  - gaze_target: geometry_msgs::msg::PointStamped::SharedPtr
     *  - ik_link_name: string [optional]
     *  - timeout: float [Default 1.0 second]
     *  We provide 2 output ports
     *  - joint_state: sensor_msgs::msg::JointState::SharedPtr
     *  - body_pose: geometry_msgs::msg::PoseStamped::SharedPtr
     */
    static BT::PortsList providedPorts(); 

    /** 
     * Make the the IK request. 
     * @return RUNNING if the IK server is available, FAILURE otherwise
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

protected:
    rclcpp::Client<spot_msgs::srv::InverseKinematics>::SharedPtr spot_ik_client_;
    rclcpp::Time query_start_time_;
    rclcpp::Duration query_timeout_{0, 0};

    std::optional<rclcpp::Client<spot_msgs::srv::InverseKinematics>::FutureAndRequestId> query_;
};

} // namespace spot_behaviors
