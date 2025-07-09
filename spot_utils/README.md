# Spot Utils

This package provides utility classes and functions for working with Boston Dynamics Spot, with a focus on camera integration and image handling.

- `camera_client.hpp` – Provides a simple interface for subscribing to specific Spot camera topics and retrieving images in ROS and base64 formats.

## Usage Example

```cpp
// Initialize camera clients
using namespace spot_utils;
using enum CameraName;

const std::vector camera_names = {FRONTLEFT, FRONTRIGHT};
auto camera_clients = initialize_camera_clients_(node_, camera_names);

auto frontleft_client = lookup_camera_client(camera_clients, FRONTLEFT);
auto frontright_client = lookup_camera_client(camera_clients, FRONTRIGHT);

// Get images
sensor_msgs::msg::Image frontleft_ros_img = frontleft_client->get_ros_image();
std::string frontright_base64_img = frontright_client->get_base64_image();
```
