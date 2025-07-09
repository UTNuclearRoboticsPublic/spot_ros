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
  std::string camera_ns;   // Namespace for the camera topic
  float img_rot_angle_rad; // Angle to rotate the image by to get it erect.
                           // Units in radians
};

class CameraClient {
public:
  std::string camera_ns;
  // Constructor initializes subscriptions and transform listener
  explicit CameraClient(const rclcpp::Node::SharedPtr &node,
                        const CamInfo &camera_info);

  sensor_msgs::msg::Image take_snapshot();

  void destroy_subscription();

  // Rotates an image to be erect
  sensor_msgs::msg::Image
  transform_image(const sensor_msgs::msg::Image &ros_image);

private:
  rclcpp::Node::SharedPtr node_; // Node
  rclcpp::Logger node_logger_;   // Node logger
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
      img_sub_;                   // Image subscription
  sensor_msgs::msg::Image image_; // Latest image
  float img_rot_angle_rad_;       // Angle to rotate the images by, in radians.
  float img_scale_factor_ = 1.0;
  int img_height_;
  int img_width_;

  // Callback for image subscription
  void img_sub_cb_(const sensor_msgs::msg::Image::SharedPtr msg);

}; // class CameraClient
} // namespace spot_utils

#endif // CAMERA_CLIENT_HPP
