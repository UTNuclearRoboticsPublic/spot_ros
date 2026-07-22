////////////////////////////////////////////////////////////////////////////////////////////
//      Title     : spot_behaviors.hpp
//      Project   : spot_ros
////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <map>
#include <filesystem>
#include <rclcpp/logging.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <nrg_utility_behaviors/trigger_service.hpp>

#include "spot_behaviors/check_arm_stowed.hpp"
#include "spot_behaviors/check_battery.hpp"
#include "spot_behaviors/check_hand_collision.hpp"
#include "spot_behaviors/dock_robot.hpp"
#include "spot_behaviors/move_hand_through_poses.hpp"
#include "spot_behaviors/move_hand_to_pose.hpp"
#include "spot_behaviors/walk_to_pose.hpp"
#include "spot_behaviors/get_stow_state.hpp"
#include "spot_behaviors/get_spot_ik.hpp"
#include "spot_behaviors/toggle_payload.hpp"
#include "spot_behaviors/get_images.hpp"
#include "spot_behaviors/get_gripper_holding_state.hpp"
#include "spot_behaviors/set_simulated_pose.hpp"
#include "spot_behaviors/navigate_to.hpp"

namespace spot_behaviors {

void registerSpotBehaviors(BT::BehaviorTreeFactory& factory, tf2_ros::Buffer::SharedPtr tf_buffer) {
    if (!tf_buffer) {
        throw std::runtime_error("Cannot register spot behaviors will a null pointer to tf_buffer!");
    }

    factory.registerNodeType<GetStowState>("GetStowState");
    factory.registerNodeType<GetGripperHoldingState>("GetGripperHoldingState");
    factory.registerNodeType<GetImages>("GetImages");
    factory.registerNodeType<SetSimulatedPose>("SetSimulatedPose");

    // Register all of the behaviors in the package
    #define REGSITER_SPOT_BEHAVIOR(name) factory.registerNodeType<name>(#name, tf_buffer)
    REGSITER_SPOT_BEHAVIOR(CheckArmStowed);
    REGSITER_SPOT_BEHAVIOR(CheckBattery);
    REGSITER_SPOT_BEHAVIOR(CheckHandCollision);
    REGSITER_SPOT_BEHAVIOR(DockRobot);
    REGSITER_SPOT_BEHAVIOR(MoveHandThroughPoses);
    REGSITER_SPOT_BEHAVIOR(WalkToPose);
    REGSITER_SPOT_BEHAVIOR(GetSpotIK);
    REGSITER_SPOT_BEHAVIOR(TogglePayload);
    REGSITER_SPOT_BEHAVIOR(NavigateTo);
    factory.registerNodeType<MoveHandToPose>("MoveSpotHandToPose", tf_buffer);

    // A manifest of subtrees and their requirements
    static const std::map<std::string, std::vector<std::string>> subtree_requirements{
        {"safely_stow_arm.xml" , {"MoveSpotHandToPose", "TriggerService", "CheckArmStowed"}},
        {"move_to.xml"         , {"NavigateToPose", "WalkToPose"}},
        {"move_hand_exact.xml" , {"MoveSpotHandToPose"}}
    };

    // Register all the sub-trees in the package
    const std::filesystem::path share_path = ament_index_cpp::get_package_share_directory("spot_behaviors");
    for (const auto& [tree_file, requirements] : subtree_requirements) {
        // Check to make sure the requirements are satisfied
        bool has_all_requirements = true;
        for (const std::string& requirement : requirements) {
            if (!factory.manifests().contains(requirement)) {
                has_all_requirements = false;
                RCLCPP_WARN(
                    rclcpp::get_logger("registerSpotBehaviors"), 
                    "Missing requirement \"%s\" for package subtree \"%s\". Subtree not registered", 
                    requirement.c_str(), tree_file.c_str()
                );
            }
        }

        // We can only register if all requirements are satisfied, otherwise we get a runtime error
        RCLCPP_INFO(rclcpp::get_logger("registerSpotBehaviors"), "Registering behavior tree file '%s'", tree_file.c_str());
        if (has_all_requirements) factory.registerBehaviorTreeFromFile(share_path/"behavior_trees"/tree_file);
    }
}

} // namespace spot_behaviors
