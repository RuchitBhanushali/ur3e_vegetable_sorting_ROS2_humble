#include "ur3e_pick_place_mtc/pick_place_task.h"

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace ur3e_pick_place_mtc {

namespace mtc = moveit::task_constructor;

static geometry_msgs::msg::PoseStamped makePoseKeepOrientation(
  const std::string& frame_id,
  const geometry_msgs::msg::Quaternion& q,
  double x, double y, double z)
{
  geometry_msgs::msg::PoseStamped ps;
  ps.header.frame_id = frame_id;
  ps.pose.orientation = q;
  ps.pose.position.x = x;
  ps.pose.position.y = y;
  ps.pose.position.z = z;
  return ps;
}

// Convert a desired TCP pose into an IK-frame (eef_frame) pose.
// Assumes TCP is located at +Z of eef_frame by tcp_offset_z.
static geometry_msgs::msg::PoseStamped tcpPoseToEefPose(
  const geometry_msgs::msg::PoseStamped& tcp_pose,
  double tcp_offset_z)
{
  if (tcp_offset_z == 0.0)
    return tcp_pose;

  geometry_msgs::msg::PoseStamped eef_pose = tcp_pose;

  tf2::Quaternion q;
  tf2::fromMsg(tcp_pose.pose.orientation, q);
  tf2::Matrix3x3 R(q);

  const tf2::Vector3 offset_local(0.0, 0.0, tcp_offset_z);
  const tf2::Vector3 offset_world = R * offset_local;

  eef_pose.pose.position.x -= offset_world.x();
  eef_pose.pose.position.y -= offset_world.y();
  eef_pose.pose.position.z -= offset_world.z();

  return eef_pose;
}

mtc::Task buildPickPlaceTask(
  const rclcpp::Node::SharedPtr& node,
  moveit::planning_interface::MoveGroupInterface& arm_mgi,
  const PickPlaceParams& p)
{
  auto pipeline = std::make_shared<mtc::solvers::PipelinePlanner>(node, "ompl");

  auto cartesian = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian->setMaxVelocityScalingFactor(0.2);
  cartesian->setMaxAccelerationScalingFactor(0.2);
  cartesian->setStepSize(0.002);

  mtc::Task task;
  task.stages()->setName("pick_place");
  task.loadRobotModel(node);

  task.setProperty("group", p.arm_group);
  task.setProperty("ik_frame", p.eef_frame);

  // ---- keep TCP orientation from the *current* robot state (prevents IK goal failures) ----
  const auto current_tcp = arm_mgi.getCurrentPose(p.eef_frame);
  const auto q_tcp = current_tcp.pose.orientation;

  // Targets (TCP poses in world)
  const auto pregrasp_tcp = makePoseKeepOrientation(p.world_frame, q_tcp, p.cube_x, p.cube_y, p.pregrasp_z);
  const auto preplace_tcp = makePoseKeepOrientation(p.world_frame, q_tcp, p.bin_x,  p.bin_y,  p.preplace_z);

  // MTC goals are specified for the IK frame (eef_frame). If your real TCP is
  // in front of that link, shift the goal pose back by tcp_offset_z.
  const auto pregrasp_pose = tcpPoseToEefPose(pregrasp_tcp, p.tcp_offset_z);
  const auto preplace_pose = tcpPoseToEefPose(preplace_tcp, p.tcp_offset_z);

  // 1) current state
  task.add(std::make_unique<mtc::stages::CurrentState>("current"));

  // 2) move to pregrasp
  {
    auto s = std::make_unique<mtc::stages::MoveTo>("move_to_pregrasp", pipeline);
    s->setGroup(p.arm_group);
    s->setIKFrame(p.eef_frame);
    s->setGoal(pregrasp_pose);
    task.add(std::move(s));
  }

  // 3) approach down (world -Z)
  {
    auto s = std::make_unique<mtc::stages::MoveRelative>("approach", cartesian);
    s->setGroup(p.arm_group);
    s->setIKFrame(p.eef_frame);

    geometry_msgs::msg::Vector3Stamped dir;
    dir.header.frame_id = p.eef_frame;
    dir.vector.z = -1.0;
    s->setDirection(dir);
    s->setMinMaxDistance(0.005, 0.08);
    task.add(std::move(s));
  }

  // 4) attach cube
  {
    auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("attach_cube");
    s->attachObject("pick_cube", p.eef_frame);
    task.add(std::move(s));
  }

  // 5) lift up (world +Z)
  {
    auto s = std::make_unique<mtc::stages::MoveRelative>("lift", cartesian);
    s->setGroup(p.arm_group);
    s->setIKFrame(p.eef_frame);

    geometry_msgs::msg::Vector3Stamped dir;
    dir.header.frame_id = p.world_frame;
    dir.vector.z = 1.0;
    s->setDirection(dir);
    s->setMinMaxDistance(0.10, 0.18);
    task.add(std::move(s));
  }

  // 6) move to preplace
  {
    auto s = std::make_unique<mtc::stages::MoveTo>("move_to_preplace", pipeline);
    s->setGroup(p.arm_group);
    s->setIKFrame(p.eef_frame);
    s->setGoal(preplace_pose);
    task.add(std::move(s));
  }

  // 7) lower down (world -Z)
  {
    auto s = std::make_unique<mtc::stages::MoveRelative>("lower", cartesian);
    s->setGroup(p.arm_group);
    s->setIKFrame(p.eef_frame);

    geometry_msgs::msg::Vector3Stamped dir;
    dir.header.frame_id = p.world_frame;
    dir.vector.z = -1.0;
    s->setDirection(dir);
    s->setMinMaxDistance(0.08, 0.16);
    task.add(std::move(s));
  }

  // 8) detach cube
  {
    auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("detach_cube");
    s->detachObject("pick_cube", p.eef_frame);
    task.add(std::move(s));
  }

  // 9) retreat up
  {
    auto s = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian);
    s->setGroup(p.arm_group);
    s->setIKFrame(p.eef_frame);

    geometry_msgs::msg::Vector3Stamped dir;
    dir.header.frame_id = p.world_frame;
    dir.vector.z = 1.0;
    s->setDirection(dir);
    s->setMinMaxDistance(0.10, 0.18);
    task.add(std::move(s));
  }

  return task;
}

}  // namespace ur3e_pick_place_mtc
