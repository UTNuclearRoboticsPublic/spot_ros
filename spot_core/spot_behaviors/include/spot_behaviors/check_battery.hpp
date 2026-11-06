////////////////////////////////////////////////////////////////////////////////////////////
//      Title     : check_battery.hpp
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

#pragma once
#include <thread>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>

#include "spot_msgs/msg/battery_state_array.hpp"
#include "spot_behaviors/node_behavior_base.hpp"

namespace spot_behaviors {

class CheckBattery : public BT::SyncActionNode, public NodeBehaviorBase {
public:
    CheckBattery(const std::string& name, const BT::NodeConfiguration& config, tf2_ros::Buffer::SharedPtr tf_buffer);

    static BT::PortsList providedPorts();

    // Returns SUCCESS if battery is over a given threshold, FAILURE otherwise
    BT::NodeStatus tick() override;

private:
    // Subscriber to data
    rclcpp::Subscription<spot_msgs::msg::BatteryStateArray>::SharedPtr battery_sub_;

    // How much battery percentage is left
    std::optional<float> battery_percentage_;

    // Record the battery state obtained from the message
    void batteryCallback(spot_msgs::msg::BatteryStateArray::UniquePtr msg);
};

} // namespace spot_behaviors
