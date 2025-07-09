/**
 * @author  Janak Panthi (Crasun Jans)
 * @brief   Utility class for subscribing to Boston Dynamics Spot's camera image
 * topics and capturing snapshots. Makes available erect images in various
 * desired formats.
 */
#ifndef CAMERA_CLIENT_HPP
#define CAMERA_CLIENT_HPP

#include "base64/base64.hpp"
#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sstream>
#include <string>
namespace spot_utils {

// Enum to hold BD Spot's Camera names
enum class CameraName { FRONTLEFT, FRONTRIGHT, LEFT, RIGHT, BACK, HAND };

// Enum to hold BD Spot's camera namespace and image rotation info
struct CamInfo {
  CameraName camera_name;  // Enum name of the camera
  std::string camera_ns;   // Namespace for the camera topic
  float img_rot_angle_rad; // Angle to rotate the image by to get it erect.
                           // Units in radians
};

class CameraClient {
public:
  std::string camera_ns;  // Namespace for the camera topic
  CameraName camera_name; // Enum name of the camera
  // Constructor initializes subscriptions and transform listener
  explicit CameraClient(const rclcpp::Node::SharedPtr &node,
                        const CamInfo &camera_info);

  sensor_msgs::msg::Image get_ros_image();
  std::string get_base64_image();
  std::string get_base64_image_url();

  void destroy_subscription();

private:
  rclcpp::Node::SharedPtr node_; // Node
  rclcpp::Logger node_logger_;   // Node logger
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
      img_sub_;                   // Image subscription
  sensor_msgs::msg::Image image_; // Latest image
  int image_lookup_timeout_secs_ = 10;
  float img_rot_angle_rad_; // Angle to rotate the images by, in radians.
  float img_scale_factor_ = 1.0;
  int img_height_;
  int img_width_;
  sensor_msgs::msg::Image
  transform_image_(const sensor_msgs::msg::Image &ros_image);
  void img_sub_cb_(const sensor_msgs::msg::Image::SharedPtr msg);
  std::string convert_mat_to_base64_(const cv::Mat &input,
                                     const std::string &imageFormat);
  std::string convert_msg_to_base64_(const sensor_msgs::msg::Image &ros_image);

}; // class CameraClient
std::vector<std::shared_ptr<CameraClient>>
initialize_camera_clients_(const std::shared_ptr<rclcpp::Node> &node,
                           const std::vector<CameraName> &camera_list);
CamInfo get_camera_info(const CameraName &camera_name);
std::shared_ptr<CameraClient> lookup_camera_client;
} // namespace spot_utils

#endif // CAMERA_CLIENT_HPP
