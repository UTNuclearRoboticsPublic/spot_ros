////////////////////////////////////////////////////////////////////////////////////////////
//      Title     : dock_robot.cpp
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

#include "spot_behaviors/dock_robot.hpp"

namespace spot_behaviors {

DockRobot::DockRobot(const std::string& name, const BT::NodeConfiguration& config, tf2_ros::Buffer::SharedPtr tf_buffer):
    BT::StatefulActionNode(name, config),
    NodeBehaviorBase(name, tf_buffer)
{
    dock_client_ = this->create_client<spot_msgs::srv::Dock>("/spot_driver/dock");
}

BT::PortsList DockRobot::providedPorts() {
    return {
        BT::InputPort<float>("dock_id")
    };
}

BT::NodeStatus DockRobot::onStart() {
    if (!dock_client_->wait_for_service(std::chrono::seconds(2))){
        RCLCPP_ERROR(get_logger(), "Server \"%s\" not found, aborting", dock_client_->get_service_name());
        return BT::NodeStatus::FAILURE;
    }

    if (service_future_.has_value()){
        RCLCPP_ERROR(get_logger(), "Dock service called with an existing dock request. This should never happen");
        return BT::NodeStatus::FAILURE;
    }

    auto request = std::make_shared<spot_msgs::srv::Dock::Request>();
    request->dock_id = getInput<int>("dock_id").value_or(520);
    RCLCPP_INFO(get_logger(), "Calling dock request to Dock ID %u", request->dock_id);\
    
    service_future_ = dock_client_->async_send_request(request);
    request_timestamp_ = now();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DockRobot::onRunning() {
    if (!service_future_.has_value()) {
        RCLCPP_WARN(get_logger(), "Spot Dock behavior running without an active request. This should never happen");
        return BT::NodeStatus::FAILURE;
    }

    const auto status = rclcpp::spin_until_future_complete(this->get_node_base_interface(), service_future_.value(), std::chrono::milliseconds(5));
    switch (status){
        case rclcpp::FutureReturnCode::SUCCESS:{
            spot_msgs::srv::Dock::Response::SharedPtr result = service_future_->get();
            service_future_ = std::nullopt;
            return result->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }

        default:
        case rclcpp::FutureReturnCode::TIMEOUT:{
            const double elapsed_seconds = (now() - request_timestamp_).seconds();
            if (elapsed_seconds > 25.0){
                RCLCPP_ERROR(get_logger(), "Timed out waiting for Dock server to respond. Aborting Dock behavior");
                dock_client_->remove_pending_request(service_future_.value());
                service_future_ = std::nullopt;
                return BT::NodeStatus::FAILURE;
            }
            else return BT::NodeStatus::RUNNING;
        }

        case rclcpp::FutureReturnCode::INTERRUPTED:
            RCLCPP_WARN(get_logger(), "Spot Dock request interrupted. Reporting failure");
            return BT::NodeStatus::FAILURE;
    }
}

void DockRobot::onHalted() {
    if (service_future_.has_value()){
        dock_client_->remove_pending_request(*service_future_);
        service_future_ = std::nullopt;
    }
}

} // namespace spot_behaviors
