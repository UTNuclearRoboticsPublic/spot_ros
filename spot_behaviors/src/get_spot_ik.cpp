#include "spot_behaviors/get_spot_ik.hpp"

namespace spot_behaviors {

GetSpotIK::GetSpotIK(const std::string& name, const BT::NodeConfiguration& config) :
StatefulActionNode(name, config),
NodeBehaviorBase(name, nullptr)
{
    spot_ik_client_ = create_client<spot_msgs::srv::InverseKinematics>("/spot_manipulation_driver/solve_ik");
}

BT::PortsList GetSpotIK::providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::PoseStamped::SharedPtr>("goal_pose", "The desired pose of the IK link"),
        BT::InputPort<geometry_msgs::msg::PointStamped::SharedPtr>("gaze_target", "Whether this goal pose represents a gaze target"),
        BT::InputPort<sensor_msgs::msg::JointState::SharedPtr>("nominal_joint_state", "The optional seed positions for some joints"),
        BT::InputPort<std::string>("ik_link_name", "arm0_hand", "The name of the link for which we want to solve IK. Default to the group end effector"),
        BT::InputPort<float>("timeout", 1.0, "How long to wait for the request to return in seconds"),
        BT::OutputPort<sensor_msgs::msg::JointState::SharedPtr>("joint_state", "The solution joint state for the robot arm only"),
        BT::OutputPort<geometry_msgs::msg::Pose::SharedPtr>("body_pose", "The pose of the robot body in the IK solution")
    };
}

BT::NodeStatus GetSpotIK::onStart() {    
    // Retrieve all the required ports
    auto goal_pose_expected = getInput<geometry_msgs::msg::PoseStamped::SharedPtr>("goal_pose");
    if (!goal_pose_expected.has_value() || !goal_pose_expected.value()) {
        RCLCPP_ERROR(get_logger(), "Missing required input port [goal_pose]");
        return BT::NodeStatus::FAILURE;
    }

    // Gaze target is optional with no default value
    auto gaze_target_expected = getInput<geometry_msgs::msg::PointStamped::SharedPtr>("gaze_target");
    const bool use_gaze_target = gaze_target_expected.has_value();

    // We still have to check default-value ports in case there is a type mismatch
    auto nominal_joint_state = std::make_shared<sensor_msgs::msg::JointState>();
    if (!getInput("nominal_joint_state", nominal_joint_state)) {
        RCLCPP_INFO(get_logger(), "Using current robot state as IK seed");
    }
    
    std::string ik_link_name = "arm0_hand";
    if (!getInput("ik_link_name", ik_link_name)) {
        RCLCPP_WARN(get_logger(), "Error on input port [ik_link_name], using default value %s", ik_link_name.c_str());
    }

    float timeout = 1.0;
    if (!getInput("timeout", timeout)) {
        RCLCPP_WARN(get_logger(), "Error on input port [timeout], using default value %.2f", timeout);
    }

    bool gaze_target = false;
    if (!getInput("gaze_target", gaze_target)) {
        RCLCPP_WARN(get_logger(), "Error on input port [gaze_target], using default value [%s]", gaze_target ? "true" : "false");
    }

    // Start the timer
    query_start_time_ = now();
    query_timeout_ = rclcpp::Duration::from_seconds(timeout);

    // Make sure the server can be reached
    if (!spot_ik_client_->wait_for_service(std::chrono::duration<float>(timeout))) {
        RCLCPP_ERROR(get_logger(), "Unable to contact %s server within the time limit. Aborting", spot_ik_client_->get_service_name());
        return BT::NodeStatus::FAILURE;
    }

    // Create the request
    auto req = std::make_shared<spot_msgs::srv::InverseKinematics::Request>();
    req->target_pose = *goal_pose_expected.value();
    req->joint_names = nominal_joint_state->name;
    req->joint_nominal_positions = nominal_joint_state->position;
    req->tool_frame = ik_link_name;
    if (use_gaze_target) {
        req->use_gaze_target = true;
        req->gaze_target = *gaze_target_expected.value();
    }

    query_ = spot_ik_client_->async_send_request(req);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GetSpotIK::onRunning() {
    // If there is no existing query, return failure - this should never happen
    if (!query_.has_value()) {
        RCLCPP_ERROR(get_logger(), "Trigger request called on service %s has no active future. This should never happen", spot_ik_client_->get_service_name());
        return BT::NodeStatus::FAILURE;
    }

    // Check the status of the future. If it's done, return its success value
    const rclcpp::Duration time_elapsed = now() - query_start_time_;
    const rclcpp::FutureReturnCode status = rclcpp::spin_until_future_complete(this->get_node_base_interface(), query_.value(), std::chrono::milliseconds(5));

    switch (status){
        default:
        case rclcpp::FutureReturnCode::TIMEOUT:
            if (time_elapsed > query_timeout_){
                RCLCPP_ERROR(get_logger(), "Service timeout. Failed to get IK result");
                onHalted();
                return BT::NodeStatus::FAILURE;
            }
            return BT::NodeStatus::RUNNING;

        case rclcpp::FutureReturnCode::INTERRUPTED:
            RCLCPP_ERROR(get_logger(), "IK request interrupted");
            onHalted();
            return BT::NodeStatus::FAILURE;

        case rclcpp::FutureReturnCode::SUCCESS:
            spot_msgs::srv::InverseKinematics::Response::SharedPtr resp = query_.value().get();
            if (!resp->solution_found) {
                RCLCPP_WARN(get_logger(), "Spot IK request returned no solution");
                return BT::NodeStatus::FAILURE;
            }
            setOutput("joint_state", std::make_shared<sensor_msgs::msg::JointState>(resp->arm_joint_state));
            setOutput("body_pose", std::make_shared<geometry_msgs::msg::Pose>(resp->body_pose));
            return BT::NodeStatus::SUCCESS;
    }
}

void GetSpotIK::onHalted() {
    if (query_.has_value()) {
        spot_ik_client_->remove_pending_request(query_.value());
        query_ = std::nullopt;
    }
}
    
} // namespace spot_behaviors
