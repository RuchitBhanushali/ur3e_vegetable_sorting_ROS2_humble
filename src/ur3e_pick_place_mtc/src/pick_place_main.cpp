#include <rclcpp/rclcpp.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include "ur3e_pick_place_mtc/task_common.h"
#include "ur3e_pick_place_mtc/scene_utils.h"
#include "ur3e_pick_place_mtc/pick_place_task.h"

namespace ur3e_pick_place_mtc {

static PickPlaceParams loadParams(const rclcpp::Node::SharedPtr& node)
{
  PickPlaceParams p;

  // Declare (YAML will override at declaration time if node name matches)
  node->declare_parameter<std::string>("arm_group",   p.arm_group);
  node->declare_parameter<std::string>("eef_frame",   p.eef_frame);
  node->declare_parameter<std::string>("world_frame", p.world_frame);

  node->declare_parameter<double>("tcp_offset_z", p.tcp_offset_z);

  node->declare_parameter<double>("cube_x", p.cube_x);
  node->declare_parameter<double>("cube_y", p.cube_y);
  node->declare_parameter<double>("cube_z", p.cube_z);

  node->declare_parameter<double>("bin_x", p.bin_x);
  node->declare_parameter<double>("bin_y", p.bin_y);
  node->declare_parameter<double>("bin_z", p.bin_z);

  node->declare_parameter<double>("pregrasp_z", p.pregrasp_z);
  node->declare_parameter<double>("preplace_z", p.preplace_z);

  // Read
  node->get_parameter("arm_group",   p.arm_group);
  node->get_parameter("eef_frame",   p.eef_frame);
  node->get_parameter("world_frame", p.world_frame);

  node->get_parameter("tcp_offset_z", p.tcp_offset_z);

  node->get_parameter("cube_x", p.cube_x);
  node->get_parameter("cube_y", p.cube_y);
  node->get_parameter("cube_z", p.cube_z);

  node->get_parameter("bin_x", p.bin_x);
  node->get_parameter("bin_y", p.bin_y);
  node->get_parameter("bin_z", p.bin_z);

  node->get_parameter("pregrasp_z", p.pregrasp_z);
  node->get_parameter("preplace_z", p.preplace_z);

  return p;
}

}  // namespace ur3e_pick_place_mtc

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // IMPORTANT: must match YAML key "ur3e_pick_place_mtc:"
  auto node = rclcpp::Node::make_shared("ur3e_pick_place_mtc");

  auto p = ur3e_pick_place_mtc::loadParams(node);

  moveit::planning_interface::MoveGroupInterface arm_mgi(node, p.arm_group);
  moveit::planning_interface::PlanningSceneInterface psi;

  // cube_z/bin_z treated as GROUND height (your YAML comment)
  ur3e_pick_place_mtc::addCubeAndBinOnlyOnGround(
    psi, p.world_frame,
    p.cube_x, p.cube_y, p.cube_z,
    p.bin_x,  p.bin_y,  p.bin_z);

  auto task = ur3e_pick_place_mtc::buildPickPlaceTask(node, arm_mgi, p);

  try {
    if (!task.plan(1)) {
      RCLCPP_ERROR(node->get_logger(), "Planning failed");
      rclcpp::shutdown();
      return 2;
    }
    task.introspection().publishSolution(*task.solutions().front());
    task.execute(*task.solutions().front());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    rclcpp::shutdown();
    return 3;
  }

  rclcpp::shutdown();
  return 0;
}
