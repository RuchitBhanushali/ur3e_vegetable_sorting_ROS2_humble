#pragma once

#include <string>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>

namespace ur3e_pick_place_mtc {

moveit_msgs::msg::CollisionObject makeBox(
  const std::string& id,
  const std::string& frame_id,
  double x, double y, double z,
  double sx, double sy, double sz);

// NOTE: cube_z and bin_z are treated as GROUND height (z of the ground surface).
void addCubeAndBinOnlyOnGround(
  moveit::planning_interface::PlanningSceneInterface& psi,
  const std::string& world_frame,
  double cube_x, double cube_y, double cube_z,
  double bin_x, double bin_y, double bin_z);

}  // namespace ur3e_pick_place_mtc
