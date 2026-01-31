#include <memory>
#include <chrono>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

// Optional (Gazebo link attacher) – keep includes if you use these services
#include <linkattacher_msgs/srv/attach_link.hpp>
#include <linkattacher_msgs/srv/detach_link.hpp>

using namespace std::chrono_literals;

class PickAndPlace
{
public:
  explicit PickAndPlace(const rclcpp::Node::SharedPtr& node)
  : node_(node),
    move_group_(node_, "ur_arm"),
    gripper_(node_, "gripper"),
    psi_(),
    logger_(rclcpp::get_logger("PickAndPlace"))
  {
    // Planning frame
    move_group_.setPoseReferenceFrame("base_link");

    // Attach/detach services (optional)
    attach_client_ = node_->create_client<linkattacher_msgs::srv::AttachLink>("/ATTACHLINK");
    detach_client_ = node_->create_client<linkattacher_msgs::srv::DetachLink>("/DETACHLINK");

    // Identify end-effector link (IMPORTANT: do NOT target wrist_3_link)
    eef_link_ = move_group_.getEndEffectorLink();
    if (eef_link_.empty()) {
      // Fallback: change to your real tool link if needed (tool0 / tcp_link)
      eef_link_ = "tool0";
      RCLCPP_WARN(logger_, "End-effector link not found from SRDF. Falling back to '%s'.",
                  eef_link_.c_str());
    } else {
      RCLCPP_INFO(logger_, "Using end-effector link: %s", eef_link_.c_str());
    }

    // Basic planner config (tune once, reused)
    move_group_.setPlanningTime(5.0);
    move_group_.allowReplanning(false);
    move_group_.setNumPlanningAttempts(10);
    move_group_.setPlanningPipelineId("ompl");
    move_group_.setPlannerId("RRTConnectkConfigDefault");

    move_group_.setMaxVelocityScalingFactor(0.6);
    move_group_.setMaxAccelerationScalingFactor(0.6);
  }

  void add_collision_objects_from_world()
  {
    std::vector<moveit_msgs::msg::CollisionObject> objs;
    objs.reserve(2);

    // pickup_table
    // {
    //   moveit_msgs::msg::CollisionObject co;
    //   co.id = "pickup_table";
    //   co.header.frame_id = "base_link";

    //   shape_msgs::msg::SolidPrimitive prim;
    //   prim.type = prim.BOX;
    //   prim.dimensions = {0.79, 0.59, 0.001};

    //   geometry_msgs::msg::Pose p;
    //   p.orientation.w = 1.0;
    //   p.position.x = 0.0;
    //   p.position.y = 0.555;
    //   p.position.z = 0.0859;

    //   co.primitives.push_back(prim);
    //   co.primitive_poses.push_back(p);
    //   co.operation = co.ADD;
    //   objs.push_back(co);
    // }

    // cube as collision object (NOTE: if cube is moving in Gazebo, this will be wrong)
    // {
    //   moveit_msgs::msg::CollisionObject co;
    //   co.id = "cube";
    //   co.header.frame_id = "base_link";

    //   shape_msgs::msg::SolidPrimitive prim;
    //   prim.type = prim.BOX;
    //   prim.dimensions = {0.04, 0.04, 0.04};

    //   geometry_msgs::msg::Pose p;
    //   p.orientation.w = 1.0;
    //   p.position.x = -0.13;
    //   p.position.y = 0.375;
    //   p.position.z = 0.1060;

    //   co.primitives.push_back(prim);
    //   co.primitive_poses.push_back(p);
    //   co.operation = co.ADD;
    //   objs.push_back(co);
    // }

    psi_.applyCollisionObjects(objs);
    RCLCPP_INFO(logger_, "Added %zu collision objects.", objs.size());
  }

  // --- Gripper control (MoveIt group) ---
  void open_gripper()
  {
    gripper_.setStartStateToCurrentState();
    gripper_.setJointValueTarget("finger_joint", 0.0);
    auto ok = (gripper_.move() == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(logger_, "Gripper open: %s", ok ? "OK" : "FAILED");
  }

  void close_gripper(double finger_joint_value = 0.5)
  {
    gripper_.setStartStateToCurrentState();
    gripper_.setJointValueTarget("finger_joint", finger_joint_value);
    auto ok = (gripper_.move() == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(logger_, "Gripper close: %s", ok ? "OK" : "FAILED");
  }

  // --- Main sequence ---
  void pick_and_place()
  {
    open_gripper();
    rclcpp::sleep_for(500ms);

    // 1) Move above the cube using normal planning (with orientation constraint)
    geometry_msgs::msg::Pose above_pick = make_pose(
      /*x=*/-0.131, /*y=*/0.370, /*z=*/0.386,
      /*roll=*/-3.124, /*pitch=*/0.0, /*yaw=*/0.0
    );
    if (!plan_and_execute_pose(above_pick)) return;

    // 2) Straight down a small Z distance (Cartesian) to avoid IK flips
    if (!cartesian_delta_z(/*dz=*/-0.06)) return;

    // 3) Close gripper
    close_gripper(0.5);
    rclcpp::sleep_for(300ms);

    // Optional attach in Gazebo (enable if you use link attacher)
    // attachObject();

    // 4) Straight up
    if (!cartesian_delta_z(/*dz=*/+0.06)) return;

    // 5) Move to place pose (example)
    geometry_msgs::msg::Pose above_place = make_pose(
      /*x=*/-0.133, /*y=*/0.370, /*z=*/0.348,
      /*roll=*/-3.124, /*pitch=*/0.0, /*yaw=*/0.0
    );
    if (!plan_and_execute_pose(above_place)) return;

    // 6) Open gripper
    open_gripper();
    rclcpp::sleep_for(300ms);

    // Optional detach
    // detachObject();
  }

private:
  // Create pose with RPY
  static geometry_msgs::msg::Pose make_pose(double x, double y, double z,
                                            double roll, double pitch, double yaw)
  {
    geometry_msgs::msg::Pose p;
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    p.orientation = tf2::toMsg(q);
    p.position.x = x;
    p.position.y = y;
    p.position.z = z;
    return p;
  }

  // Apply an orientation constraint to prevent wrist/elbow flips
  void set_orientation_constraint(const geometry_msgs::msg::Quaternion& q)
  {
    moveit_msgs::msg::OrientationConstraint oc;
    oc.header.frame_id = "base_link";
    oc.link_name = eef_link_;
    oc.orientation = q;

    // tighten/loosen as needed
    oc.absolute_x_axis_tolerance = 0.25;
    oc.absolute_y_axis_tolerance = 0.25;
    oc.absolute_z_axis_tolerance = 0.40;
    oc.weight = 1.0;

    moveit_msgs::msg::Constraints c;
    c.orientation_constraints.push_back(oc);
    move_group_.setPathConstraints(c);
  }

  void clear_constraints()
  {
    move_group_.clearPathConstraints();
  }

  bool plan_and_execute_pose(const geometry_msgs::msg::Pose& target)
  {
    move_group_.setStartStateToCurrentState();

    // Prevent flips
    set_orientation_constraint(target.orientation);

    move_group_.setGoalPositionTolerance(0.01);
    move_group_.setGoalOrientationTolerance(0.05);

    move_group_.setPoseTarget(target, eef_link_);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool ok = (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(logger_, "Plan pose: %s", ok ? "SUCCESS" : "FAILED");

    if (!ok) {
      clear_constraints();
      return false;
    }

    auto exec = move_group_.execute(plan);
    clear_constraints();

    bool exec_ok = (exec == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(logger_, "Execute pose: %s", exec_ok ? "OK" : "FAILED");
    return exec_ok;
  }

  // Straight cartesian move in Z to avoid OMPL detours
  bool cartesian_delta_z(double dz)
  {
    move_group_.setStartStateToCurrentState();

    // Keep current orientation fixed during cartesian motion
    auto current = move_group_.getCurrentPose(eef_link_).pose;
    set_orientation_constraint(current.orientation);

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.reserve(2);

    geometry_msgs::msg::Pose p1 = current;
    geometry_msgs::msg::Pose p2 = current;
    p2.position.z += dz;

    waypoints.push_back(p1);
    waypoints.push_back(p2);

    moveit_msgs::msg::RobotTrajectory traj;
    // eef_step: 1cm, jump_threshold: 0 (disable jump check)
    double fraction = move_group_.computeCartesianPath(waypoints, 0.01, 0.0, traj, true);
    RCLCPP_INFO(logger_, "Cartesian fraction: %.2f", fraction);

    if (fraction < 0.95) {
      clear_constraints();
      RCLCPP_ERROR(logger_, "Cartesian path incomplete (fraction < 0.95).");
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = traj;

    auto exec = move_group_.execute(plan);
    clear_constraints();

    bool exec_ok = (exec == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(logger_, "Execute cartesian: %s", exec_ok ? "OK" : "FAILED");
    return exec_ok;
  }

  // Optional Gazebo attach/detach
  void attachObject()
  {
    auto req = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();
    req->model1_name = "ur";
    req->link1_name  = "wrist_3_link";     // if your plugin requires this exact link
    req->model2_name = "cube_pick";
    req->link2_name  = "link_1";

    if (!attach_client_->wait_for_service(2s)) {
      RCLCPP_ERROR(logger_, "AttachLink service not available.");
      return;
    }

    auto fut = attach_client_->async_send_request(req);
    if (rclcpp::spin_until_future_complete(node_, fut) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(logger_, "Attached object.");
    } else {
      RCLCPP_ERROR(logger_, "Attach failed.");
    }
  }

  void detachObject()
  {
    auto req = std::make_shared<linkattacher_msgs::srv::DetachLink::Request>();
    req->model1_name = "ur";
    req->link1_name  = "wrist_3_link";
    req->model2_name = "cube_pick";
    req->link2_name  = "link_1";

    if (!detach_client_->wait_for_service(2s)) {
      RCLCPP_ERROR(logger_, "DetachLink service not available.");
      return;
    }

    auto fut = detach_client_->async_send_request(req);
    if (rclcpp::spin_until_future_complete(node_, fut) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(logger_, "Detached object.");
    } else {
      RCLCPP_ERROR(logger_, "Detach failed.");
    }
  }

private:
  rclcpp::Node::SharedPtr node_;

  moveit::planning_interface::MoveGroupInterface move_group_;
  moveit::planning_interface::MoveGroupInterface gripper_;
  moveit::planning_interface::PlanningSceneInterface psi_;

  rclcpp::Client<linkattacher_msgs::srv::AttachLink>::SharedPtr attach_client_;
  rclcpp::Client<linkattacher_msgs::srv::DetachLink>::SharedPtr detach_client_;

  rclcpp::Logger logger_;
  std::string eef_link_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pick_and_place_node");

  PickAndPlace demo(node);
  demo.add_collision_objects_from_world();
  rclcpp::sleep_for(500ms);

  demo.pick_and_place();

  rclcpp::shutdown();
  return 0;
}
