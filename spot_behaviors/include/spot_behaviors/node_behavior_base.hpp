#pragma once

#include <cctype>
#include <string>
#include <memory>
#include <optional>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace spot_behaviors{

class NodeBehaviorBase : public rclcpp::Node {
public:
    NodeBehaviorBase(const std::string& name, tf2_ros::Buffer::SharedPtr tf_buffer, std::string node_namespace = "spot_behaviors"):
    rclcpp::Node(toUniqueName(name), node_namespace),
    tf_buffer_{tf_buffer}
    {}

    // Check if a transform can be looked up safely within a set time period
    bool checkTransform(const std::string& source_frame, const std::string& target_frame, rclcpp::Time timepoint = rclcpp::Time{0}, rclcpp::Duration timeout = std::chrono::milliseconds(500)) const{
        std::string err;
        if (!tf_buffer_->canTransform(target_frame, source_frame, timepoint, timeout, &err)){
            RCLCPP_ERROR(get_logger(), "%s", err.c_str());
            return false;
        }
        return true;
    }

    std::optional<Eigen::Isometry3d> getFramePose(std::string target_frame, std::string task_frame) const {
        if (!checkTransform(target_frame, task_frame)) return std::nullopt;
        return tf2::transformToEigen( 
            tf_buffer_->lookupTransform(
                task_frame, 
                target_frame, 
                rclcpp::Time(0)
            )
        );
    }

    static std::string toSnakeCase(const std::string& str) {
        std::string snake_str;
        auto isUpper = [](char c) {return c >= 'A' && c <= 'Z';};
        
        for (const char& c : str) {
            const char& c_next = *std::next(&c); 
            snake_str.push_back(std::tolower((unsigned char)(c)));
            if (!isUpper(c) && isUpper(c_next)) {
                snake_str.push_back('_');
            }
        }

        return snake_str;
    }

    static std::string toUniqueName(const std::string& name) {
        std::string unique_name = toSnakeCase(name).append("_");
        name_mapping_.try_emplace(unique_name, 0);
        name_mapping_[unique_name]++;
        unique_name.append(std::to_string(name_mapping_[unique_name]-1));
        return unique_name;
    }

protected:
    tf2_ros::Buffer::SharedPtr tf_buffer_;
    inline static std::unordered_map<std::string, int> name_mapping_;
};

} // namespace alpha_survey_3d
