#include <optional>
#include <behaviortree_cpp/action_node.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <spot_msgs/action/navigate_to.hpp>
#include <spot_behaviors/node_behavior_base.hpp>

namespace spot_behaviors {

class NavigateTo : public BT::StatefulActionNode, public NodeBehaviorBase {
public:
    NavigateTo(const std::string& name, const BT::NodeConfig& config, tf2_ros::Buffer::SharedPtr tf_buffer);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    // Configuration
    float client_timeout_;
    float navigation_timeout_;
    std::string action_name_;

    // Action Client
    rclcpp_action::Client<spot_msgs::action::NavigateTo>::SharedPtr navigation_client_;
    rclcpp_action::Client<spot_msgs::action::NavigateTo>::GoalHandle::SharedPtr goal_handle_;
    std::shared_future<decltype(goal_handle_)> goal_handle_future_;
    std::optional<spot_msgs::action::NavigateTo::Result> result_;

    // Bookkeeping
    rclcpp::Time request_start_time_;
    rclcpp::Time navigation_start_time_;
    bool response_received_ = false;
};

} // namespace spot_behaviors