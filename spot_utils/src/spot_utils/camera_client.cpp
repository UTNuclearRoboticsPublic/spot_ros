#include "spot_utils/camera_client.hpp"
namespace spot_utils {

// -- Start of Camera Client Methods -- //
CameraClient::CameraClient(const rclcpp::Node::SharedPtr &node,
                           const CamInfo &camera_info)
    : camera_name(camera_info.camera_name), camera_ns(camera_info.camera_ns),
      node_(node), node_logger_(node->get_logger()),
      img_rot_angle_rad_(camera_info.img_rot_angle_rad) {
  img_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
      camera_ns + "/image", 10,
      std::bind(&CameraClient::img_sub_cb_, this, std::placeholders::_1));
}

// Retrieves ROS image from the camera client
sensor_msgs::msg::Image CameraClient::get_ros_image() {
  using namespace std::chrono_literals;

  std::chrono::seconds img_lookup_timeout_secs(
      img_lookup_timeout_secs_); // Configurable timeout

  auto start_time = std::chrono::steady_clock::now();
  rclcpp::Rate loop_rate(10); // 10 Hz

  while (image_.encoding.empty()) {
    rclcpp::spin_some(node_);
    loop_rate.sleep();

    // Check timeout
    auto current_time = std::chrono::steady_clock::now();
    if (current_time - start_time > img_lookup_timeout_secs) {
      throw std::runtime_error(
          "Failed to get images within allotted timeout for camera: " +
          this->camera_ns);
    }
  }

  return transform_image_(image_);
}

// Retrieves base64 image from the camera client
std::string CameraClient::get_base64_image() {

  // Lookup ROS image
  auto img = this->get_ros_image();

  // Return converted image
  return convert_msg_to_base64_(img);
}

// Retrieves base64 image url (i.e. MIME type prefix added to base64 encoded
// image) from the camera client
std::string CameraClient::get_base64_image_url() {
  const std::string base64_image = this->get_base64_image();
  return "data:image/" + img_format_ + ";base64," + base64_image;
}

void CameraClient::destroy_subscription() {

  img_sub_.reset(); // Image subscription
}

// Rotates an image to be erect
sensor_msgs::msg::Image
CameraClient::transform_image_(const sensor_msgs::msg::Image &ros_image) {

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

// Convert cv::Mat to a base64-encoded string with a specified format
std::string
CameraClient::convert_mat_to_base64_(const cv::Mat &input,
                                     const std::string &img_format_) {
  // Encode the cv::Mat to a specified image format (e.g., JPEG, PNG)
  std::vector<unsigned char> buffer;
  std::vector<int> params;

  // Set encoding parameters for quality, if needed (e.g., JPEG quality)
  if (img_format_ == "jpeg" || img_format_ == "jpg")
    params = {cv::IMWRITE_JPEG_QUALITY, 95}; // Adjust quality as needed
  else if (img_format_ == "png")
    params = {cv::IMWRITE_PNG_COMPRESSION, 3}; // Adjust compression as needed

  if (!cv::imencode("." + img_format_, input, buffer, params)) {
    throw std::runtime_error("Failed to encode image to format: " +
                             img_format_);
  }

  // Convert the binary buffer to a base64 string
  std::string encodedImage =
      base64::to_base64(std::string(buffer.begin(), buffer.end()));

  if (encodedImage.empty()) {
    throw std::runtime_error("Base64 image is empty!");
  }

  return encodedImage;
}
std::string
CameraClient::convert_msg_to_base64_(const sensor_msgs::msg::Image &ros_image) {
  try {
    // Create a shared_ptr msg from the image
    const auto msg = std::make_shared<sensor_msgs::msg::Image>(ros_image);

    // Convert ROS2 Image message to OpenCV image (in RGB format)
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8); // Assuming all incoming ROS msgs have this encoding

    // Convert RGB (ROS standard) to BGR (OpenCV default) to avoid color shift
    cv::Mat bgr_image;
    cv::cvtColor(cv_ptr->image, bgr_image, cv::COLOR_RGB2BGR);

    return convert_mat_to_base64_(bgr_image, img_format_);
  } catch (cv_bridge::Exception &e) {
    throw std::runtime_error(std::string("Error during image conversion: ") + e.what());
  }
}

// -- EOF of Camera Client Methods -- //

std::vector<std::shared_ptr<CameraClient>>
initialize_camera_clients_(const std::shared_ptr<rclcpp::Node> &node,
                           const std::vector<CameraName> &camera_list) {

  std::vector<std::shared_ptr<CameraClient>> camera_client_list;
  camera_client_list.reserve(camera_list.size());
  try {
    for (const auto &camera_name : camera_list) {
      CamInfo cam_info = get_camera_info(camera_name);
      camera_client_list.emplace_back(
          std::make_shared<CameraClient>(node, cam_info));
    }
  } catch (const std::exception &e) {
    std::cerr << "Caught exception during camera client initialization: "
              << e.what() << std::endl;
    return {}; // Return empty vector on failure
  }

  return camera_client_list;
}

CamInfo get_camera_info(const CameraName &camera_name) {
  CamInfo camera_info;
  camera_info.camera_name = camera_name;

  switch (camera_name) {
  case CameraName::FRONTLEFT:
    camera_info.camera_ns = "/spot_image_server/rgb/frontleft";
    camera_info.img_rot_angle_rad = -M_PI / 2.0;
    break;
  case CameraName::FRONTRIGHT:
    camera_info.camera_ns = "/spot_image_server/rgb/frontright";
    camera_info.img_rot_angle_rad = -M_PI / 2.0;
    break;
  case CameraName::LEFT:
    camera_info.camera_ns = "/spot_image_server/rgb/left";
    camera_info.img_rot_angle_rad = 0;
    break;
  case CameraName::RIGHT:
    camera_info.camera_ns = "/spot_image_server/rgb/right";
    camera_info.img_rot_angle_rad = M_PI;
    break;
  case CameraName::BACK:
    camera_info.camera_ns = "/spot_image_server/rgb/back";
    camera_info.img_rot_angle_rad = 0;
    break;
  case CameraName::HAND:
    camera_info.camera_ns = "/spot_image_server/rgb/hand_color";
    camera_info.img_rot_angle_rad = 0;
    break;
  }
  return camera_info;
}

std::shared_ptr<CameraClient> lookup_camera_client(
    const std::vector<std::shared_ptr<CameraClient>> &camera_client_list,
    const CameraName &camera_name) {

  for (const auto &camera_client : camera_client_list) {
    if (camera_client && camera_client->camera_name == camera_name) {
      return camera_client;
    }
  }

  return nullptr; // Return nullptr if not found
}

} // namespace spot_utils
