#include <span>
#include <mutex>
#include <thread>
#include <ranges>
#include <cstring>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <rcpputils/endian.hpp>
#include <eigen3/Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <spot_msgs/msg/manipulator_stow_state.hpp>

namespace spot_navigation {

class PointcloudFilterComponent : public rclcpp::Node {
public:
    PointcloudFilterComponent(rclcpp::NodeOptions opts = rclcpp::NodeOptions{}) :
    Node("spot_pointcloud_filter", opts),
    tf_buffer(get_clock()),
    tf_listener(tf_buffer)
    {
        sensor_frame_ = declare_parameter("sensor_frame", "velodyne");
        critical_links.back() = declare_parameter("tool_frame", "arm0_hand");

        // Use a one-shot timer to initialize so we don't block the constructor
        init_timer_ = create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&PointcloudFilterComponent::initializeFilter, this));

        using namespace std::placeholders;
        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
        pointcloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_out", 10);
        pointcloud_sub = create_subscription<sensor_msgs::msg::PointCloud2>("cloud_in", 10, 
                std::bind(&PointcloudFilterComponent::filterPointcloud, this, _1), sub_opts);

        stow_state_sub_ = create_subscription<spot_msgs::msg::ManipulatorStowState>("/spot_manipulation_driver/manipulator_state/stow_state", 1, 
            [this](spot_msgs::msg::ManipulatorStowState::SharedPtr state){arm_stowed_ = state->state == state->STOWSTATE_STOWED;});
    }

    void initializeFilter() {
        init_timer_->cancel(); // Only run once

        try {
            // Check if transform exists without a long blocking wait in constructor context
            if (!tf_buffer.canTransform(sensor_frame_, "arm0_base_link", tf2::TimePointZero, std::chrono::seconds(5))) {
                RCLCPP_WARN(get_logger(), "Waiting for transform between %s and arm0_base_link...", sensor_frame_.c_str());
                init_timer_ = create_wall_timer(std::chrono::seconds(2), std::bind(&PointcloudFilterComponent::initializeFilter, this));
                return;
            }

            const geometry_msgs::msg::TransformStamped sensor_tform_arm_base = tf_buffer.lookupTransform(
                sensor_frame_,
                "arm0_base_link",
                tf2::TimePointZero
            );

            if (std::abs(sensor_tform_arm_base.transform.rotation.w) < 0.95) {
                RCLCPP_ERROR(get_logger(), "Your lidar frame %s is not aligned with your robot.", sensor_frame_.c_str());
                return; 
            }
        } catch (tf2::TransformException& e) {
            RCLCPP_ERROR(get_logger(), "TF Initialization failed: %s", e.what());
            return;
        }

        // Start periodic updates using a ROS timer 
        update_timer_ = create_wall_timer(
            std::chrono::milliseconds(250),
            std::bind(&PointcloudFilterComponent::updateTransforms, this));

        RCLCPP_INFO(get_logger(), "Spot pointcloud filter online");
    }

    void updateTransforms() {
        try {
            for (std::size_t link_idx = 0; link_idx < critical_links.size(); link_idx++) {
                const std::string& frame = critical_links[link_idx];
                geometry_msgs::msg::TransformStamped sensor_tform_link = tf_buffer.lookupTransform(
                    sensor_frame_,
                    frame,
                    tf2::TimePointZero
                );
                tf2::fromMsg(sensor_tform_link.transform.translation, critical_link_locs[link_idx]);
            }

            {
                std::lock_guard guard(arm_update_mtx_);
                arm_min_pt.x() = std::ranges::min_element(critical_link_locs, {}, [](auto& pt){return pt.x();})->x();
                arm_min_pt.y() = std::ranges::min_element(critical_link_locs, {}, [](auto& pt){return pt.y();})->y();
                arm_min_pt.z() = std::ranges::min_element(critical_link_locs, {}, [](auto& pt){return pt.z();})->z();
                arm_max_pt.x() = std::ranges::max_element(critical_link_locs, {}, [](auto& pt){return pt.x();})->x();
                arm_max_pt.y() = std::ranges::max_element(critical_link_locs, {}, [](auto& pt){return pt.y();})->y();
                arm_max_pt.z() = std::ranges::max_element(critical_link_locs, {}, [](auto& pt){return pt.z();})->z();

                // Add a buffer since frame locations are at the link centers
                arm_max_pt += 0.10 * Eigen::Vector3d::Ones();
                arm_min_pt -= 0.10 * Eigen::Vector3d::Ones();
                transforms_ready_ = true;
            }
        } catch (tf2::TransformException& e) {
            RCLCPP_DEBUG(get_logger(), "Could not update arm transforms: %s", e.what());
        }
    }

    void filterPointcloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud) {
        if (!transforms_ready_ && !arm_stowed_) return;

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
            static constexpr float body_threshold = 35.0f * deg2rad;
            static constexpr float body_elevation_threshold = -25.0f * deg2rad;
            static constexpr float arm_threshold = 7.5f * deg2rad;
            static constexpr float arm_base_elevation_threshold = -14.0f * deg2rad;
            static constexpr float arm_base_threshold = 18.5f * deg2rad;
            static constexpr float lidar_cable_threshold = (180.0f - 22.5f) * deg2rad;
            static constexpr float lidar_elevation_threshold = -20.0f * deg2rad;
            
            if (z < 0) {
                // Check for arm obstruction
                const float azimuthal_angle = std::abs(std::atan2(y, x));
                if (azimuthal_angle < arm_threshold) {
                    accept_point = false;
                } 
                
                const Eigen::Vector3f ray_dir = Eigen::Vector3f(x, y, z).normalized();
                const float elevation_angle = std::asin(ray_dir.z());

                // Frontal obstructions (arm and forward legs)
                if (x > 0) {
                    if (azimuthal_angle < body_threshold) {
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
                // Rear obstructions (lidar cable)
                else {
                    if ((azimuthal_angle > lidar_cable_threshold) && (elevation_angle < lidar_elevation_threshold)) {
                        accept_point = false;
                    }
                }
            } 

            // If the arm is deployed, we also need to check against that
            // For computationaly feasibility, we just check against the entire arm AABB
            if (accept_point && !arm_stowed_) {
                const Eigen::Array3d ray_dir = Eigen::Vector3d(x, y, z).normalized().array();
                std::lock_guard guard(arm_update_mtx_);

                // Ray-BoundingBox intersection algorithm
                double tmin = -std::numeric_limits<float>::infinity();
                double tmax = std::numeric_limits<float>::infinity();
                for (std::size_t idx = 0; idx < 3; idx++){  // x, y, z
                    // Check for the special case of zero values
                    if (ray_dir[idx] == 0.0) {
                        if (arm_min_pt[idx] > 0 || arm_max_pt[idx] < 0){
                            break;
                        } else {
                            continue;
                        }
                    }

                    // Get the scalar distance to the bounding box on this axis 
                    double t1 = arm_min_pt[idx] / ray_dir[idx];
                    double t2 = arm_max_pt[idx] / ray_dir[idx];
                    auto [tmin_new, tmax_new] = std::minmax(t1, t2);

                    tmin = std::max(tmin, tmin_new);
                    tmax = std::min(tmax, tmax_new);
                }

                if (tmin < tmax && tmax > 0){
                    accept_point = false;
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
    rclcpp::Subscription<spot_msgs::msg::ManipulatorStowState>::SharedPtr stow_state_sub_;
    
    rclcpp::TimerBase::SharedPtr init_timer_;
    rclcpp::TimerBase::SharedPtr update_timer_;

    std::string tool_frame_; // Optional tool frame if something is attached to the end effector
    std::string sensor_frame_;  // LiDAR frame
    std::array<std::string, 4> critical_links {
        "arm0_base_link",
        "arm0_wrist_roll",
        "arm0_elbow_pitch"
    };
    std::array<Eigen::Vector3d, 4> critical_link_locs{};
    bool arm_stowed_ = true;
    bool transforms_ready_ = false;

    std::mutex arm_update_mtx_;
    Eigen::Vector3d arm_min_pt;
    Eigen::Vector3d arm_max_pt;
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
