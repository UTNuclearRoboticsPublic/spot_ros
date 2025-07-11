/**
 * @file    get_images.hpp
 * @author  Janak Panthi (Crasun Jans)
 */

#ifndef GET_IMAGES_HPP
#define GET_IMAGES_HPP

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <spot_utils/camera_client.hpp>
#include <string>
#include <behaviortree_cpp/bt_factory.h>
#include <sstream>

namespace BT {

/**
 * @brief .
 *
 * This function is a template specialization of BT::convertFromString for the
 * spot_utils::CameraName enum. It trims whitespace and attempts to match the input
 * string (e.g., "FRONTLEFT") to a valid enum entry using the STRING_TO_CAMERA_NAME map.
 *
 * @param str Input string view from the BehaviorTree XML.
 * @return spot_utils::CameraName The corresponding enum value.
 *
 * @throws BT::RuntimeError If the input string does not match any known CameraName.
 */
template <>
inline spot_utils::CameraName convertFromString(StringView str)
{
  std::string token{ str.data(), str.size() };

  // Trim whitespace
  token.erase(0, token.find_first_not_of(" \t"));
  token.erase(token.find_last_not_of(" \t") + 1);

  auto it = spot_utils::STRING_TO_CAMERA_NAME.find(token);
  if (it != spot_utils::STRING_TO_CAMERA_NAME.end()) {
    return it->second;
  }

  throw BT::RuntimeError("Unknown CameraName: ", token);
}

/**
 * @brief Parses a list of CameraName enum values from BehaviorTree.CPP XML input to a vector of CameraName enums.
 *
 * Supports input in comma- or semicolon-separated form, with optional braces.
 * Example accepted formats:
 *   - "FRONTLEFT, FRONTRIGHT"
 *   - "FRONTLEFT; FRONTRIGHT"
 *   - "{FRONTLEFT, FRONTRIGHT}"
 *
 * @param str The input string representing a list of camera names.
 * @return std::vector of spot_utils::CameraName values.
 * @throws BT::RuntimeError if any token is invalid.
 */
template <> inline
std::vector<spot_utils::CameraName> convertFromString(BT::StringView str)
{
  std::vector<spot_utils::CameraName> out;

  std::string cleaned{ str.data(), str.size() };

  // Trim surrounding whitespace
  cleaned.erase(0, cleaned.find_first_not_of(" \t\n\r"));
  cleaned.erase(cleaned.find_last_not_of(" \t\n\r") + 1);

  // If input is wrapped in { }, remove them
  if (!cleaned.empty() && cleaned.front() == '{' && cleaned.back() == '}') {
    cleaned = cleaned.substr(1, cleaned.size() - 2);
  }

  std::stringstream ss{ cleaned };
  std::string token;

  // Detect delimiter
  const char delim = (cleaned.find(',') != std::string::npos) ? ',' : ';';

  while (std::getline(ss, token, delim))
  {
    // Trim whitespace
    token.erase(0, token.find_first_not_of(" \t\n\r"));
    token.erase(token.find_last_not_of(" \t\n\r") + 1);

    // Re-use the single enum converter
    out.push_back(convertFromString<spot_utils::CameraName>(token));
  }

  return out;
}
} // namespace BT


namespace spot_behaviors {

/**
 * @brief Behavior Tree node that captures images from specified Spot cameras.
 *
 * This node receives a list of camera identifiers and uses Spot's camera client utilities
 * to fetch the latest images from each camera. It outputs the images in both ROS and base64
 * formats along with their corresponding identifiers.
 */
class GetImages : public BT::SyncActionNode {
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

private:
  std::shared_ptr<rclcpp::Node> node_; ///< Shared ptr to the ROS node
};

} // namespace spot_behaviors

#endif // GET_IMAGES_HPP
