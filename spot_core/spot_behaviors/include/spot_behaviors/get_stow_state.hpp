/**
 * @file    get_stow_state.hpp
 * @author  Janak Panthi (Crasun Jans)
 */

#ifndef GET_STOW_STATE_HPP
#define GET_STOW_STATE_HPP

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <spot_msgs/msg/manipulator_stow_state.hpp>

namespace spot_behaviors {

/**
 * @brief Behavior Tree node that queries Spot to determine if the arm is stowed.
 */
class GetStowState : public BT::SyncActionNode {
public:
  /**
   * @brief Constructs the GetStowState node.
   * @param name   Name of the Behavior Tree node.
   * @param config Configuration object for the node.
   */
  explicit GetStowState(const std::string &name, const BT::NodeConfig &config);

  /**
   * @brief Defines the ports used by this node.
   *
   * **Input port:**  
   * `timeout_secs` (`double`) — Optional timeout to read the state in seconds.
   *
   * **Output port:**  
   * `is_stowed` (`bool`) — True if the arm is stowed.
   *
   * @return List of declared input/output ports.
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Checks the manipulator state to determine if the arm is stowed.
   * @return `SUCCESS` if the state was read successfully; `FAILURE` otherwise.
   */
  BT::NodeStatus tick() override;

private:
  std::shared_ptr<rclcpp::Node> node_; ///< ROS node handle.
  rclcpp::Subscription<spot_msgs::msg::ManipulatorStowState>::SharedPtr stow_state_sub_; ///< Subscriber to the stow state
  static constexpr const char* stow_state_topic_name_ = "/spot_manipulation_driver/manipulator_state/stow_state";
  bool is_stowed_ = false; ///< Whether the arm is stowed
  bool stow_state_received_ = false; ///< Whether subscription was successful
  double default_timeout_sec_ = 1.0; ///< How long to read the state


};

}  // namespace spot_behaviors

#endif  // GET_STOW_STATE_HPP

