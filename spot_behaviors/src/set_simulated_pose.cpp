#include "spot_behaviors/set_simulated_pose.hpp"

namespace spot_behaviors {

SetSimulatedPose::SetSimulatedPose(
    const std::string& name,
    const BT::NodeConfiguration& config
) : 
BT::StatefulActionNode(name, config),
NodeBehaviorBase(name, nullptr)
{}

BT::PortsList SetSimulatedPose::providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::PoseStamped::SharedPtr>("pose", "The pose at which to place the simulated robot"),
        BT::InputPort<float>("timeout", 5.0f, "The number of seconds to wait for a response from the server"),
        BT::InputPort<std::string>("service_name", "/spot_simulation/set_robot_pose", "The fully qualified name of the server to call"),
        BT::InputPort<std::string>("robot_name", "spot", "The name of the robot for which to set the pose")
    };
}

BT::NodeStatus SetSimulatedPose::onStart() {
    if (std::string service_name; getInput("service_name", service_name)) {
        pose_set_client_ = create_client<spot_msgs::srv::SetSimulatedPose>(service_name);
    } else {
        RCLCPP_ERROR(get_logger(), "Unable to retrieve the name of the server");
        return BT::NodeStatus::FAILURE;
    }

    float timeout;
    if (!getInput("timeout", timeout)) {
        RCLCPP_ERROR(get_logger(), "Unable to retrieve service call timeout");
        return BT::NodeStatus::FAILURE;
    }

    query_start_time_ = now();
    if (!pose_set_client_->wait_for_service(std::chrono::duration<float>(timeout))) {
        RCLCPP_ERROR(get_logger(), "Unable to contact the server within the time limit");
        return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::PoseStamped::SharedPtr robot_pose;
    if (!getInput("pose", robot_pose) || !robot_pose) {
        RCLCPP_ERROR(get_logger(), "Unable to retrieve robot pose from blackboard");
        return BT::NodeStatus::FAILURE;
    }
    
    std::string robot_name;
    if (!getInput("robot_name", robot_name)) {
        RCLCPP_ERROR(get_logger(), "Unable to retrieve robot name from blackboard");
        return BT::NodeStatus::FAILURE;
    }
    
    auto req = std::make_shared<spot_msgs::srv::SetSimulatedPose::Request>();
    req->pose = *robot_pose;
    req->robot_name = robot_name;

    auto callback = [this](std::shared_future<spot_msgs::srv::SetSimulatedPose::Response::SharedPtr> response_future) {pose_set_response_ = response_future.get();};
    pose_set_client_->async_send_request(req, callback);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SetSimulatedPose::onRunning() {
    rclcpp::spin_some(this->get_node_base_interface());

    if (pose_set_response_) {
        const bool success = pose_set_response_->success;
        pose_set_response_.reset();
        if (!success) {
            RCLCPP_WARN(get_logger(), "Failed to set the pose of the simulated robot");
            return BT::NodeStatus::FAILURE;
        } else {
            return BT::NodeStatus::SUCCESS;
        }
    }

    const float timeout = getInput<float>("timeout").value();
    if ((now() - query_start_time_).seconds() > timeout) {
        RCLCPP_WARN(get_logger(), "Did not receive a response from the server within the time limit");
        onHalted();
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}

void SetSimulatedPose::onHalted() {
    pose_set_response_.reset();
    if (pose_set_client_) {
        pose_set_client_->prune_pending_requests();
    }
}

} // namespace spot_behaviors