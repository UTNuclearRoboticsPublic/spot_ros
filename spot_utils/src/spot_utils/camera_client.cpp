#include "spot_utils/camera_client.hpp"
namespace spot_utils {
CameraClient::CameraClient(const rclcpp::Node::SharedPtr &node,
                           const CamInfo &camera_info)
    : node_(node), node_logger_(node->get_logger()),
      img_rot_angle_rad_(camera_info.img_rot_angle_rad),
      camera_ns(camera_info.camera_ns)

{
  img_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
      camera_ns + "/image", 10,
      std::bind(&CameraClient::img_sub_cb_, this, std::placeholders::_1));
}

// Takes a snapshot of the current image and camera information
sensor_msgs::msg::Image CameraClient::take_snapshot() { return image_; }

void CameraClient::destroy_subscription() {

  img_sub_.reset(); // Image subscription
}

// Rotates an image to be erect
sensor_msgs::msg::Image
CameraClient::transform_image(const sensor_msgs::msg::Image &ros_image) {

  // Create a shared_ptr msg from the image
  const auto msg = std::make_shared<sensor_msgs::msg::Image>(ros_image);

  // Convert ROS2 Image message to OpenCV image
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);

  // Get OpenCV Mat from cv_bridge
  const cv::Mat image = cv_ptr->image;

  // Capture image dimensions
  img_height_ = image.rows;
  img_width_ = image.cols;
  float img_rot_angle_deg = (180 / M_PI) * img_rot_angle_rad_;

  // Define rotation matrix
  const cv::Point2f center(img_width_ / 2.0, img_height_ / 2.0);
  cv::Mat rotation_matrix =
      cv::getRotationMatrix2D(center, img_rot_angle_deg, img_scale_factor_);

  double angle_rad = img_rot_angle_rad_;
  double cos_angle = std::abs(std::cos(angle_rad));
  double sin_angle = std::abs(std::sin(angle_rad));

  int new_width =
      static_cast<int>(img_width_ * cos_angle + img_height_ * sin_angle);
  int new_height =
      static_cast<int>(img_width_ * sin_angle + img_height_ * cos_angle);

  // Adjust the rotation matrix to account for the new size
  rotation_matrix.at<double>(0, 2) += (new_width - img_width_) / 2.0;
  rotation_matrix.at<double>(1, 2) += (new_height - img_height_) / 2.0;

  // Create a matrix for the rotated image
  cv::Mat rotated_image;

  // Rotate the image with the new size
  cv::warpAffine(image, rotated_image, rotation_matrix,
                 cv::Size(new_width, new_height));

  // Convert rotated image back to ROS2 Image message
  cv_bridge::CvImage out_img;
  out_img.header = msg->header;
  out_img.encoding = sensor_msgs::image_encodings::RGB8;
  out_img.image = rotated_image;

  return *out_img.toImageMsg();
}

// Callback for image subscription
void CameraClient::img_sub_cb_(const sensor_msgs::msg::Image::SharedPtr msg) {
  image_ = *msg;
}

} // namespace spot_utils
