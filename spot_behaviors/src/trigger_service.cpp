////////////////////////////////////////////////////////////////////////////////////////////
//      Title     : tigger_service.cpp
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

#include "spot_behaviors/trigger_service.hpp"

namespace spot_behaviors {

TriggerService::TriggerService(const std::string& name, const BT::NodeConfiguration& config):
    BT::StatefulActionNode(name, config),
    node_(std::make_shared<rclcpp::Node>(name+"BT"+std::to_string(node_count_++), "spot_behaviors"))
{}

BT::PortsList TriggerService::providedPorts() {
    return {
        BT::InputPort<std::string>("service_name", "Fully qualified name of the service to call"),
        BT::InputPort<int>("timeout", "Time in seconds to wait for a response"),
        BT::InputPort<bool>("empty", "Whether to use an Empty service type instead of a Trigger service type")
    };
}

BT::NodeStatus TriggerService::onStart() {
    BT::Expected<std::string> service_name = getInput<std::string>("service_name");
    const bool use_empty = getInput<bool>("empty").value_or(false);

    if (use_empty) {
        empty_client_ = node_->create_client<std_srvs::srv::Empty>(service_name.value());
        client_base_ = std::dynamic_pointer_cast<rclcpp::ClientBase>(empty_client_);
    }else{
        trigger_client_ = node_->create_client<std_srvs::srv::Trigger>(service_name.value());
        client_base_ = std::dynamic_pointer_cast<rclcpp::ClientBase>(trigger_client_);
    }

    // Make sure the server is available to us
    if (!client_base_->wait_for_service(std::chrono::seconds(2))){
        RCLCPP_ERROR(node_->get_logger(), "%s server \"%s\" not found, aborting", (trigger_client_ ? "Trigger" : "Empty"), service_name.value().c_str());
        return BT::NodeStatus::FAILURE;
    }

    // Make sure there aren't any existing requests - should not be possible
    if ((trigger_client_ && trigger_future_.has_value()) || (empty_client_ && empty_future_.has_value())){
        RCLCPP_ERROR(node_->get_logger(), "Service \"%s\" called with an existing request. This should never happen", service_name.value().c_str());
        return BT::NodeStatus::FAILURE;
    }

    // Update the timeout value if one has been provided
    BT::Expected<int> timeout_expected = getInput<int>("timeout");
    if (!timeout_expected.has_value()){
        RCLCPP_WARN(node_->get_logger(), "Argument \"timeout\" not passed to TriggerService behavior, using default value of 2 seconds");
    }
    timeout_ = std::chrono::milliseconds(timeout_expected.value_or(2)*1000);

    // Create and send the request
    if (trigger_client_) {
        RCLCPP_INFO(node_->get_logger(), "Calling trigger request on \"%s\"", service_name.value().c_str());
        trigger_future_ = trigger_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    }else {
        RCLCPP_INFO(node_->get_logger(), "Calling empty request on \"%s\"", service_name.value().c_str());
        empty_future_ = empty_client_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
    }
    service_call_time_ = node_->now();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TriggerService::onRunning() {
    // If there is no existing request, return failure - this should never happen
    if ( (trigger_client_ && !trigger_future_.has_value()) || (empty_client_ && !empty_future_.has_value()) ) {
        RCLCPP_ERROR(node_->get_logger(), "Trigger request called on service %s has no active future. This should never happen", client_base_->get_service_name());
        return BT::NodeStatus::FAILURE;
    }

    // Check the status of the future. If it's done, return its success value
    rclcpp::Duration time_elapsed = node_->now() - service_call_time_;
    rclcpp::FutureReturnCode status = std::invoke([&](){
        if (trigger_future_.has_value())
            return rclcpp::spin_until_future_complete(node_, trigger_future_.value(), std::chrono::milliseconds(5));
        else
            return rclcpp::spin_until_future_complete(node_, empty_future_.value(), std::chrono::milliseconds(5));
    });

    switch (status){
        case rclcpp::FutureReturnCode::TIMEOUT:
            if (time_elapsed > timeout_){
                RCLCPP_ERROR(node_->get_logger(), "Service timeout. Failed to trigger service %s", client_base_->get_service_name());
                onHalted();
                return BT::NodeStatus::FAILURE;
            }
            return BT::NodeStatus::RUNNING;

        case rclcpp::FutureReturnCode::INTERRUPTED:
            RCLCPP_ERROR(node_->get_logger(), "Service interrupted. Failed to trigger service %s", client_base_->get_service_name());
            onHalted();
            return BT::NodeStatus::FAILURE;

        case rclcpp::FutureReturnCode::SUCCESS:
            if (trigger_client_) {
                const std_srvs::srv::Trigger::Response::SharedPtr resp = trigger_future_->get();
                trigger_future_ = std::nullopt;
                if (!resp->success){
                    RCLCPP_ERROR(node_->get_logger(), "Error in service call to %s: %s", client_base_->get_service_name(), resp->message.c_str());
                    return BT::NodeStatus::FAILURE;
                }
            }else {
                empty_future_ = std::nullopt;
            }
            return BT::NodeStatus::SUCCESS;
    }

    // Otherwise return running
    RCLCPP_ERROR(node_->get_logger(), "Trigger service behavior for %s running without an active request. This should never happend", client_base_->get_service_name());
    return BT::NodeStatus::FAILURE;
}

void TriggerService::onHalted() {
    if (trigger_future_.has_value()) {
        trigger_client_->remove_pending_request(*trigger_future_);
        trigger_future_ = std::nullopt;
    }

    if (empty_future_.has_value()) {
        empty_client_->remove_pending_request(*empty_future_);
        empty_future_ = std::nullopt;
    }
}

} // namespace spot_behaviors
