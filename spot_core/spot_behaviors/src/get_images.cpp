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
      BT::OutputPort<std::shared_ptr<std::vector<sensor_msgs::msg::Image>>>("image_list")
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

  auto image_list = std::make_shared<std::vector<sensor_msgs::msg::Image>>();

  for (const auto &client : camera_clients) {
    const sensor_msgs::msg::Image& img_msg = client->get_ros_image();
    image_list->push_back(img_msg);

    client->destroy_subscription();
  }

  setOutput("image_list", image_list);
  return BT::NodeStatus::SUCCESS;
}

} // namespace spot_behaviors

