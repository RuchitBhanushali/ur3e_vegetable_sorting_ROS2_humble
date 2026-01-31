#pragma once

#include <rclcpp/node.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include "ur3e_pick_place_mtc/task_common.h"

namespace ur3e_pick_place_mtc {

moveit::task_constructor::Task buildPickPlaceTask(
  const rclcpp::Node::SharedPtr& node,
  moveit::planning_interface::MoveGroupInterface& arm_mgi,
  const PickPlaceParams& p);

}  // namespace ur3e_pick_place_mtc
