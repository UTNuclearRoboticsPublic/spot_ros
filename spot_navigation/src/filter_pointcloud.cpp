#include <span>
#include <ranges>
#include <cstring>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <rcpputils/endian.hpp>
#include <eigen3/Eigen/Geometry>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace spot_navigation {

class PointcloudFilterComponent : public rclcpp::Node {
public:
    PointcloudFilterComponent(rclcpp::NodeOptions opts = rclcpp::NodeOptions{}) :
    Node("spot_pointcloud_filter", opts),
    tf_buffer(get_clock()),
    tf_listener(tf_buffer)
    {
        const std::string sensor_frame = declare_parameter("sensor_frame", "velodyne");

        // We assume that the lidar is fixed relative to the body and is aligned with the body
        // and that it is mounted with the NRG elevated lidar mount
        // TODO: Parameterize angle limits to work with other mounts
        try {
            const geometry_msgs::msg::TransformStamped sensor_tform_body = tf_buffer.lookupTransform(
                sensor_frame,
                "body",
                tf2::TimePointZero,
                std::chrono::seconds(20)  // extra time for ouster to get online
            );

            if (std::abs(sensor_tform_body.transform.rotation.w) < 0.95) {
                RCLCPP_ERROR(get_logger(), "Your lidar frame %s is not aligned with your robot. This code wasn't meant for that", sensor_frame.c_str());
                exit(1);
            }
        } catch (tf2::TransformException& e) {
            RCLCPP_ERROR(get_logger(), e.what());
            exit(1);
        }

        using namespace std::placeholders;
        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
        pointcloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_out", 10);
        pointcloud_sub = create_subscription<sensor_msgs::msg::PointCloud2>("cloud_in", 10, 
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

            bool accept_point = true;
            
            // Check the ray direction. We don't want points pointing forward and down
            // since it hits the arm and body and creates a "bleeding points" effect
            static constexpr float deg2rad = M_PIf32 / 180.0f;
            static constexpr float body_threshold = 32.0f * deg2rad;
            static constexpr float body_elevation_threshold = -25.0f * deg2rad;
            static constexpr float arm_threshold = 7.5f * deg2rad;
            static constexpr float arm_base_elevation_threshold = -18.0f * deg2rad;
            static constexpr float arm_base_threshold = 17.0f * deg2rad;
            
            if (z < 0 && x > 0) {
                // Check for arm obstruction
                const float azimuthal_angle = std::abs(std::atan2(y, x));
                if (azimuthal_angle < arm_threshold) {
                    accept_point = false;
                } 
                else if (azimuthal_angle < body_threshold) {
                    const Eigen::Vector3f ray_dir = Eigen::Vector3f(x, y, z).normalized();
                    const float elevation_angle = std::asin(ray_dir.z());

                    // Check for body obstruction
                    if (elevation_angle < body_elevation_threshold) {
                        accept_point = false;                    
                    }
                    // Check for arm base obstruction
                    else if (azimuthal_angle < arm_base_threshold && elevation_angle < arm_base_elevation_threshold) {
                        accept_point = false;
                    }
                }
            }

            if (!accept_point) continue;
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
};

} // namespace spot_navigation

#ifdef COMPILE_AS_COMPONENT
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(spot_navigation::PointcloudFilterComponent);
#endif

#if COMPILE_AS_NODE
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<spot_navigation::PointcloudFilterComponent>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
#endif