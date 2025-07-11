#include "spot_behaviors/get_images.hpp"

namespace spot_behaviors {

GetImages::GetImages(const std::string &name, const BT::NodeConfig &config)
    : BT::SyncActionNode(name, config) 
{
  node_ = rclcpp::Node::make_shared(name);
}

BT::PortsList GetImages::providedPorts() {
  return {
      BT::InputPort<std::vector<spot_utils::CameraName>>("camera_names"),
      BT::OutputPort<std::shared_ptr<std::vector<spot_utils::StampedImage>>>("stamped_image_list")
  };
}

BT::NodeStatus GetImages::tick() {
  rclcpp::spin_some(node_);

  using CameraNameVec = std::vector<spot_utils::CameraName>;
  BT::Expected<CameraNameVec> camera_names_exp =
      getInput<CameraNameVec>("camera_names");

  if (!camera_names_exp) {
    throw BT::RuntimeError("Input [camera_names] is missing or invalid.");
  }

  CameraNameVec camera_names = camera_names_exp.value();

  auto camera_clients = spot_utils::initialize_camera_clients_(node_, camera_names);

  auto stamped_image_list = std::make_shared<std::vector<spot_utils::StampedImage>>();

  for (const auto &client : camera_clients) {
    spot_utils::StampedImage si;
    si.id = client->camera_ns;
    si.base64_image = client->get_base64_image();
    si.ros_image = client->get_ros_image();
    stamped_image_list->push_back(si);

    client->destroy_subscription();
  }

  setOutput("stamped_image_list", stamped_image_list);
  return BT::NodeStatus::SUCCESS;
}

} // namespace spot_behaviors

