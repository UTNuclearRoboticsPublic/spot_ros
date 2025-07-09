#include "spot_behaviors/get_images.hpp"

namespace spot_behaviors {

GetImages::GetImages(const std::string &name, const BT::NodeConfig &config)
    : BT::SyncActionNode(name, config), Node(name) {}

BT::PortsList GetImages::providedPorts() {
  return {
      BT::InputPort<std::shared_ptr<std::vector<spot_utils::CameraName>>>(
          "camera_names"),
      BT::OutputPort<std::shared_ptr<std::vector<spot_utils::StampedImage>>>(
          "stamped_image_list")};
}

BT::NodeStatus GetImages::tick() {
  rclcpp::spin_some(this->get_node_base_interface());

  using CameraNameVecPtr = std::shared_ptr<std::vector<spot_utils::CameraName>>;
  BT::Expected<CameraNameVecPtr> camera_names_exp =
      getInput<CameraNameVecPtr>("camera_names");

  if (!camera_names_exp) {
    throw BT::RuntimeError("Input [camera_names] is missing or invalid.");
  }

  CameraNameVecPtr camera_names = camera_names_exp.value();

  // shared_from_this() will return SyncActionNode ptr so, casting is needed to
  // get the rclcpp::Node ptr
  auto node = std::static_pointer_cast<rclcpp::Node>(this->shared_from_this());

  // Initialize camera clients
  auto camera_clients =
      spot_utils::initialize_camera_clients_(node, *camera_names);

  // Prepare output
  auto stamped_image_list =
      std::make_shared<std::vector<spot_utils::StampedImage>>();

  for (const auto &client : camera_clients) {
    spot_utils::StampedImage si;
    si.id = client->camera_ns;
    si.base64_image = client->get_base64_image();
    si.ros_image = client->get_ros_image();
    stamped_image_list->push_back(si);

    client->destroy_subscription(); // Clean up after capture
  }

  setOutput("stamped_image_list", stamped_image_list);
  return BT::NodeStatus::SUCCESS;
}

} // namespace spot_behaviors
