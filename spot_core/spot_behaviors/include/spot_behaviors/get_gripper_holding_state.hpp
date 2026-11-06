/**
 * @file    get_gripper_holding_state.hpp
 * @author  Janak Panthi (Crasun Jans)
 */

#ifndef GET_GRIPPER_HOLDING_STATE_HPP
#define GET_GRIPPER_HOLDING_STATE_HPP

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

namespace spot_behaviors {

/**
 * @brief Behavior Tree node that queries Spot’s gripper to determine if it is holding an object.
 */
class GetGripperHoldingState : public BT::SyncActionNode {
public:
  /**
   * @brief Constructs the GetGripperHoldingState node.
   * @param name   Name of the Behavior Tree node.
   * @param config Configuration object for the node.
   */
  explicit GetGripperHoldingState(const std::string &name, const BT::NodeConfig &config);

  /**
   * @brief Defines the ports used by this node.
   *
   * **Input port:**  
   * `timeout_secs` (`double`) — Optional timeout to read the state in seconds.
   *
   * **Output port:**  
   * `is_holding` (`bool`) — True if the gripper is holding an object.
   *
   * @return List of declared input/output ports.
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Checks the manipulator state to determine if the gripper is holding an object.
   * @return `SUCCESS` if the state was read successfully; `FAILURE` otherwise.
   */
  BT::NodeStatus tick() override;

private:
  std::shared_ptr<rclcpp::Node> node_; ///< ROS node handle.
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_state_sub_; ///< Subscriber to the gripper holding state
  static constexpr const char* gripper_state_topic_name_ = "/spot_manipulation_driver/manipulator_state/is_gripper_carrying_item";
  bool is_holding_ = false; ///< Whether the gripper is holding something
  bool gripper_state_received_ = false; ///< Whether subscription was successful
  double default_timeout_sec_ = 1.0; ///< How long to read the state


};

}  // namespace spot_behaviors

#endif  // GET_GRIPPER_HOLDING_STATE_HPP

