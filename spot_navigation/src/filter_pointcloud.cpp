#include <span>
#include <ranges>
#include <cstring>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <rcpputils/endian.hpp>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

struct BoundingBox {
    double x_min{};
    double x_max{};
    double y_min{};
    double y_max{};
    double z_min{};
    double z_max{};
};

namespace spot_navigation {

class PointcloudFilterComponent : public rclcpp::Node {
public:
    PointcloudFilterComponent(const rclcpp::NodeOptions& opts) :
    Node("spot_pointcloud_filter_component", opts),
    tf_buffer(get_clock()),
    tf_listener(tf_buffer)
    {
        // Parameterize the bounding box bounds, with the default value covering only the arm
        const std::string sensor_frame = declare_parameter("sensor_frame", "velodyne");
        const std::vector<double> bounding_box_min = declare_parameter("bounding_box_min_in_body", std::vector<double>({-0.20, -0.10, 0.10}));
        const std::vector<double> bounding_box_max = declare_parameter("bounding_box_max_in_body", std::vector<double>({ 0.65,  0.10, 0.40}));
        bounding_box.x_min = bounding_box_min.at(0);
        bounding_box.y_min = bounding_box_min.at(1);
        bounding_box.z_min = bounding_box_min.at(2);
        bounding_box.x_max = bounding_box_max.at(0);
        bounding_box.y_max = bounding_box_max.at(1);
        bounding_box.z_max = bounding_box_max.at(2);

        // We assume that the lidar is fixed relative to the body and is aligned with the body
        try {
            const geometry_msgs::msg::TransformStamped sensor_tform_body = tf_buffer.lookupTransform(
                sensor_frame,
                "body",
                tf2::TimePointZero,
                std::chrono::seconds(5)
            );

            if (sensor_tform_body.transform.rotation.w < 0.95) {
                RCLCPP_ERROR(get_logger(), "Your lidar is not aligned with your robot. This code wasn't meant for that.");
                exit(1);
            }

            bounding_box.x_min += sensor_tform_body.transform.translation.x;
            bounding_box.x_max += sensor_tform_body.transform.translation.x;
            bounding_box.y_min += sensor_tform_body.transform.translation.y;
            bounding_box.y_max += sensor_tform_body.transform.translation.y;
            bounding_box.z_min += sensor_tform_body.transform.translation.z;
            bounding_box.z_max += sensor_tform_body.transform.translation.z;
        } catch (tf2::TransformException& e) {
            RCLCPP_ERROR(get_logger(), e.what());
        }

        using namespace std::placeholders;
        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
        pointcloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_out", rclcpp::SensorDataQoS{});
        pointcloud_sub = create_subscription<sensor_msgs::msg::PointCloud2>("cloud_in", rclcpp::SensorDataQoS{}, 
                std::bind(&PointcloudFilterComponent::filterPointcloud, this, _1), sub_opts);

        RCLCPP_INFO(get_logger(), "Spot pointcloud filter online");
    }

    void filterPointcloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud) {
        std::ptrdiff_t x_offset, y_offset, z_offset;
        for (const sensor_msgs::msg::PointField& field : pointcloud->fields) {
            if (field.name == "x") {
                x_offset = field.offset;
            } else if (field.name == "y") {
                y_offset = field.offset;
            } else if (field.name == "z") {
                z_offset = field.offset;
            }
        }

        auto filtered_cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
        filtered_cloud->fields = pointcloud->fields;
        filtered_cloud->header = pointcloud->header;
        filtered_cloud->height = 1;
        filtered_cloud->is_bigendian = pointcloud->is_bigendian;
        filtered_cloud->is_dense = pointcloud->is_dense;
        filtered_cloud->point_step = pointcloud->point_step;
        filtered_cloud->data.reserve(pointcloud->data.size());

        const bool reverse_bytes = (!filtered_cloud->is_bigendian && rcpputils::endian::native == rcpputils::endian::big)
                                || (filtered_cloud->is_bigendian && rcpputils::endian::native == rcpputils::endian::little);
        
        std::size_t new_size = 0;
        for (auto data_ptr = pointcloud->data.begin(); data_ptr != pointcloud->data.end(); std::advance(data_ptr, pointcloud->point_step)) {
            auto point_span = std::span<const uint8_t>(data_ptr, pointcloud->point_step);

            auto x_span = point_span.subspan(x_offset, 4);
            auto y_span = point_span.subspan(y_offset, 4);
            auto z_span = point_span.subspan(z_offset, 4);

            float x, y, z;
            std::memcpy(&x, x_span.data(), 4);
            std::memcpy(&y, y_span.data(), 4);
            std::memcpy(&z, z_span.data(), 4);

            if (reverse_bytes) {
                std::reverse(reinterpret_cast<std::byte*>(&x), reinterpret_cast<std::byte*>(&x) + sizeof(float));
                std::reverse(reinterpret_cast<std::byte*>(&y), reinterpret_cast<std::byte*>(&y) + sizeof(float));
                std::reverse(reinterpret_cast<std::byte*>(&z), reinterpret_cast<std::byte*>(&z) + sizeof(float));
            }
                
            if (x < bounding_box.x_max && x > bounding_box.x_min && y < bounding_box.y_max && y > bounding_box.y_min && z < bounding_box.z_max && z > bounding_box.z_min) continue;
            new_size++;
            filtered_cloud->data.insert(filtered_cloud->data.end(), data_ptr, std::next(data_ptr, pointcloud->point_step));
        }
        
        filtered_cloud->row_step = new_size * pointcloud->point_step;
        filtered_cloud->width = new_size;
        pointcloud_pub->publish(std::move(filtered_cloud));
    }
private:
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub;

    BoundingBox bounding_box;
};

} // namespace spot_navigation

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(spot_navigation::PointcloudFilterComponent);
