/**
 * @file    camera_client.hpp
 * @author  Janak Panthi (Crasun Jans)
 * @brief   Utility class for subscribing to Boston Dynamics Spot's camera image
 *          topics and capturing snapshots. In the simplest case, one can
 *          initialize the camera clients and get desired images in two lines of
 *          code.
 */

#ifndef CAMERA_CLIENT_HPP
#define CAMERA_CLIENT_HPP

#include "base64/base64.hpp"
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sstream>
#include <string>

// Handle change from humble to jazzy
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

namespace spot_utils {

/**
 * @brief Enum to represent camera identifiers on Boston Dynamics Spot.
 */
enum class CameraName { FRONTLEFT, FRONTRIGHT, LEFT, RIGHT, BACK, HAND };

/**
* @brief Unordered Map of string to CameraName enum
*/
inline const std::unordered_map<std::string, CameraName> STRING_TO_CAMERA_NAME = {
    {"FRONTLEFT",  CameraName::FRONTLEFT},
    {"FRONTRIGHT", CameraName::FRONTRIGHT},
    {"LEFT",       CameraName::LEFT},
    {"RIGHT",      CameraName::RIGHT},
    {"BACK",       CameraName::BACK},
    {"HAND",       CameraName::HAND}
};

/**
 * @brief Struct to hold configuration for a specific Spot camera.
 */
struct CamInfo {
  CameraName camera_name;  ///< Enum name of the camera
  std::string camera_ns;   ///< ROS topic namespace for the camera image
  float img_rot_angle_rad; ///< Rotation (in radians) to make the image upright
};

/**
 * @brief Struct to hold image id, and images in base64 and sensor_msgs::msg::Image format
 */
struct StampedImage {
  std::string id;
  std::string base64_image;
  sensor_msgs::msg::Image ros_image;
};

/**
 * @class CameraClient
 * @brief Class to subscribe to Spot's camera feed and retrieve images in
 * various formats.
 *
 */
class CameraClient {
public:
  CameraName camera_name; ///< Enum name of the camera
  std::string camera_ns;  ///< Namespace for the camera topic

  /**
   * @brief Constructor that sets up the image subscription.
   * @param node Shared ROS 2 node pointer
   * @param camera_info Configuration for the target camera
   */
  explicit CameraClient(const rclcpp::Node::SharedPtr &node,
                        const CamInfo &camera_info);

  /**
   * @brief Retrieve the latest image from the camera as a ROS Image message.
   * @return sensor_msgs::msg::Image The latest image. Note a transformation is
   * applied to make images upright, which otherwise come rotated
   * characteristically from Spot.
   */
  sensor_msgs::msg::Image get_ros_image();

  /**
   * @brief Retrieve the latest camera image as a base64-encoded JPEG.
   * @return std::string base64 image data
   */
  std::string get_base64_image();

  /**
   * @brief Retrieve the base64-encoded image prepended with MIME type for URL
   * embedding.
   * @return std::string A complete data URL string
   */
  std::string get_base64_image_url();

  /**
   * @brief Manually destroy the image subscription.
   */
  void destroy_subscription();

private:
  rclcpp::Node::SharedPtr node_; ///< ROS node
  rclcpp::Logger node_logger_;   ///< Node logger
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
      img_sub_;                   ///< Image subscription
  sensor_msgs::msg::Image image_; ///< Latest received image

  const int img_lookup_timeout_secs_ = 10; ///< Timeout for waiting for an image
  const float img_scale_factor_ = 1.0;    ///< Scaling factor for image resizing
  const std::string img_format_ = "jpeg"; ///< Default format for encoding

  float img_rot_angle_rad_; ///< Angle (radians) to rotate image to upright
                            ///< position
  int img_height_;          ///< Height of the image (pixels)
  int img_width_;           ///< Width of the image (pixels)

  /**
   * @brief Rotate and adjust the image to an upright pose.
   * @param ros_image Input ROS image
   * @return sensor_msgs::msg::Image Transformed upright image
   */
  sensor_msgs::msg::Image
  transform_image_(const sensor_msgs::msg::Image &ros_image);

  /**
   * @brief Internal image subscriber callback.
   * @param msg Incoming image message
   */
  void img_sub_cb_(const sensor_msgs::msg::Image::SharedPtr msg);

  /**
   * @brief Encode OpenCV image to base64 string.
   * @param input OpenCV image matrix
   * @param img_format_ Image format (e.g., "jpeg", "png")
   * @return std::string Base64-encoded image string
   */
  std::string convert_mat_to_base64_(const cv::Mat &input,
                                     const std::string &img_format_);

  /**
   * @brief Convert ROS image message to base64 string.
   * @param ros_image Input ROS image message
   * @return std::string Base64-encoded JPEG string
   */
  std::string convert_msg_to_base64_(const sensor_msgs::msg::Image &ros_image);
};

/**
 * @brief Helper function to initialize multiple CameraClient instances.
 * @param node Shared ROS 2 node
 * @param camera_list List of cameras to initialize
 * @return std::vector<std::shared_ptr<CameraClient>> List of active camera
 * clients
 */
std::vector<std::shared_ptr<CameraClient>>
initialize_camera_clients_(const std::shared_ptr<rclcpp::Node> &node,
                           const std::vector<CameraName> &camera_list);

/**
 * @brief Get configuration info for a specific camera.
 * @param camera_name Enum identifier of the camera
 * @return CamInfo Configuration including topic namespace and rotation
 */
CamInfo get_camera_info(const CameraName &camera_name);

/**
 * @brief Find and return the matching CameraClient from a list.
 * @param camera_client_list List of available CameraClients
 * @param camera_name Target camera to search for
 * @return std::shared_ptr<CameraClient> Matching client or nullptr
 */
std::shared_ptr<CameraClient> lookup_camera_client(
    const std::vector<std::shared_ptr<CameraClient>> &camera_client_list,
    const CameraName &camera_name);

} // namespace spot_utils

#endif // CAMERA_CLIENT_HPP
