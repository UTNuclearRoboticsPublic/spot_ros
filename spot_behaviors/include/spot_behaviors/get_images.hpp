#ifndef GET_IMAGES_HPP
#define GET_IMAGES_HPP

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <spot_utils/camera_client.hpp>
#include <string>

namespace spot_behaviors {

/**
 * @brief Behavior Tree node that captures images from specified Spot cameras.
 *
 * This node receives a list of camera identifiers and uses Spot's camera client utilities
 * to fetch the latest images from each camera. It outputs the images in both ROS and base64
 * formats along with their corresponding identifiers.
 */
class GetImages : public BT::SyncActionNode, public rclcpp::Node {
public:
  /**
   * @brief Constructor for the GetImages node.
   *
   * @param name   Name of the Behavior Tree node.
   * @param config Configuration object for the Behavior Tree node.
   */
  explicit GetImages(const std::string &name, const BT::NodeConfig &config);

  /**
   * @brief Returns the list of input and output ports used by this node.
   *
   * - Input port: `camera_names`  
   *   Type: `std::shared_ptr<std::vector<spot_utils::CameraName>>`  
   *   Description: List of Spot camera enums to query.
   *
   * - Output port: `stamped_image_list`  
   *   Type: `std::shared_ptr<std::vector<spot_utils::StampedImage>>`  
   *   Description: List of images captured from the specified cameras. Each image
   *   includes the camera namespace (`id`), a ROS `sensor_msgs::msg::Image`, and
   *   a base64-encoded string of the image.
   *
   * @return PortsList containing the input and output ports.
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Executes the image capture behavior.
   *
   * This method:
   * - Retrieves the list of camera names from the input port.
   * - Initializes one-time camera clients.
   * - Captures and stores the latest image from each camera.
   * - Populates the output port with a list of stamped images.
   *
   * @return SUCCESS if all images are captured and set; FAILURE otherwise.
   */
  BT::NodeStatus tick() override;
};

} // namespace spot_behaviors

#endif // GET_IMAGES_HPP
