////////////////////////////////////////////////////////////////////////////////////////////
//      Title     : spot_behaviors.hpp
//      Project   : spot_ros
//      Copyright : Copyright© The University of Texas at Austin, 2024. All rights reserved.
//                
//          All files within this directory are subject to the following, unless an alternative
//          license is explicitly included within the text of each file.
//
//          This software and documentation constitute an unpublished work
//          and contain valuable trade secrets and proprietary information
//          belonging to the University. None of the foregoing material may be
//          copied or duplicated or disclosed without the express, written
//          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
//          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
//          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
//          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
//          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
//          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
//          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
//          University be liable for incidental, special, indirect, direct or
//          consequential damages or loss of profits, interruption of business,
//          or related expenses which may arise from use of software or documentation,
//          including but not limited to those resulting from defects in software
//          and/or documentation, or loss or inaccuracy of data of any kind.
//
////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <spot_behaviors/check_arm_stowed.hpp>
#include <spot_behaviors/check_battery.hpp>
#include <spot_behaviors/check_hand_collision.hpp>
#include <spot_behaviors/dock_robot.hpp>
#include <spot_behaviors/move_hand_through_poses.hpp>
#include <spot_behaviors/move_hand_to_pose.hpp>
#include <spot_behaviors/navigate_to_pose.hpp>
#include <spot_behaviors/record_current_location.hpp>
#include <spot_behaviors/walk_to_pose.hpp>

#define REGISTER_SPOT_BEHAVIORS(factory, tf_buffer) \
    factory.registerNodeType<spot_behaviors::CheckArmStowed>("CheckArmStowed", tf_buffer);\
    factory.registerNodeType<spot_behaviors::CheckBattery>("CheckBattery", tf_buffer);\
    factory.registerNodeType<spot_behaviors::CheckHandCollision>("CheckHandCollision", tf_buffer);\
    factory.registerNodeType<spot_behaviors::DockRobot>("DockRobot", tf_buffer);\
    factory.registerNodeType<spot_behaviors::MoveHandThroughPoses>("MoveHandThroughPoses", tf_buffer);\
    factory.registerNodeType<spot_behaviors::MoveHandToPose>("MoveHandToPose", tf_buffer);\
    factory.registerNodeType<spot_behaviors::NavigateToPose>("NavigateToPose", tf_buffer);\
    factory.registerNodeType<spot_behaviors::RecordCurrentLocation>("RecordCurrentLocation", tf_buffer);\
    factory.registerNodeType<spot_behaviors::WalkToPose>("WalkToPose", tf_buffer); \
    \ 
    factory.registerBehaviorTreeFromFile(std::filesystem::path(ament_index_cpp::get_package_share_directory("spot_behaviors")).append("behavior_trees").append("safely_stow_arm.xml")); \
    factory.registerBehaviorTreeFromFile(std::filesystem::path(ament_index_cpp::get_package_share_directory("spot_behaviors")).append("behavior_trees").append("move_to.xml"));
