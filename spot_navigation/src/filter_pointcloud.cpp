#include <span>
#include <ranges>
#include <cstring>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <rcpputils/endian.hpp>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

struct BoundingBox {
    double x_min{};
    double x_max{};
    double y_min{};
    double y_max{};
    double z_min{};
    double z_max{};
};

consteval auto generateBoundingBoxes() {
    // All bounding boxes are defined in the body frame
    BoundingBox body_bounding_box {
        .x_min =  0.00,
        .x_max =  0.80,
        .y_min = -0.40,
        .y_max =  0.40,
        .z_min = -0.50,
        .z_max =  0.15
    };

    BoundingBox arm_bounding_box {
        .x_min =  0.00,
        .x_max =  0.70,
        .y_min = -0.15,
        .y_max =  0.15,
        .z_min =  0.00,
        .z_max =  0.32
    };

    BoundingBox stray_point_box {
        .x_min =  0.60,
        .x_max =  1.50,
        .y_min = -0.25,
        .y_max =  0.25,
        .z_min = -0.57,
        .z_max = -0.30
    };

    return std::array{body_bounding_box, arm_bounding_box, stray_point_box};
}

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

            for (BoundingBox& bbox : bounding_boxes) {
                bbox.x_min += sensor_tform_body.transform.translation.x;
                bbox.x_max += sensor_tform_body.transform.translation.x;
                bbox.y_min += sensor_tform_body.transform.translation.y;
                bbox.y_max += sensor_tform_body.transform.translation.y;
                bbox.z_min += sensor_tform_body.transform.translation.z;
                bbox.z_max += sensor_tform_body.transform.translation.z;
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

        region_marker_pub = create_publisher<visualization_msgs::msg::MarkerArray>("~/exclusion_region", rclcpp::QoS(1).transient_local());
        publishRegionVisualization(sensor_frame);

        RCLCPP_INFO(get_logger(), "Spot pointcloud filter online");
    }

    void publishRegionVisualization(const std::string sensor_frame) {
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker region_marker;

        region_marker.action = region_marker.ADD;
        region_marker.color.a = 0.2f;
        region_marker.color.g = 1.0f;
        region_marker.frame_locked = true;
        region_marker.type = region_marker.CUBE;
        region_marker.header.frame_id = sensor_frame;

        for (const BoundingBox& bbox : bounding_boxes) {
            region_marker.scale.x = bbox.x_max - bbox.x_min;
            region_marker.scale.y = bbox.y_max - bbox.y_min;
            region_marker.scale.z = bbox.z_max - bbox.z_min;
            region_marker.pose.position.x = 0.5*(bbox.x_min + bbox.x_max);
            region_marker.pose.position.y = 0.5*(bbox.y_min + bbox.y_max);
            region_marker.pose.position.z = 0.5*(bbox.z_min + bbox.z_max);
            marker_array.markers.push_back(region_marker);
            region_marker.id++;
        }

        region_marker_pub->publish(marker_array);
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
            for (const BoundingBox& bbox : bounding_boxes) {
                if (x < bbox.x_max && x > bbox.x_min && y < bbox.y_max && y > bbox.y_min && z < bbox.z_max && z > bbox.z_min) {
                    accept_point = false;
                    break;
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
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr region_marker_pub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub;

    std::array<BoundingBox, 3> bounding_boxes = generateBoundingBoxes();
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