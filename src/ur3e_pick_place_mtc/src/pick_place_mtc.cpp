/**
 * @file pick_place_mtc_node.cpp
 * @brief Single-node MoveIt Task Constructor pick & place for UR3e + Robotiq.
 */

#include <rclcpp/rclcpp.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>

#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <thread>

namespace mtc = moveit::task_constructor;

namespace {

moveit_msgs::msg::CollisionObject makeBoxOnGround(
  const std::string& id,
  const std::string& frame_id,
  double ground_x, double ground_y, double ground_z,
  double size_x, double size_y, double size_z)
{
  moveit_msgs::msg::CollisionObject obj;
  obj.id = id;
  obj.header.frame_id = frame_id;

  shape_msgs::msg::SolidPrimitive prim;
  prim.type = shape_msgs::msg::SolidPrimitive::BOX;
  prim.dimensions.resize(3);
  prim.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = size_x;
  prim.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = size_y;
  prim.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = size_z;

  geometry_msgs::msg::Pose p;
  p.orientation.w = 1.0;
  p.position.x = ground_x;
  p.position.y = ground_y;
  p.position.z = ground_z + size_z * 0.5;

  obj.primitives.push_back(prim);
  obj.primitive_poses.push_back(p);
  obj.operation = moveit_msgs::msg::CollisionObject::ADD;
  return obj;
}

geometry_msgs::msg::PoseStamped makePoseKeepOrientation(
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

geometry_msgs::msg::PoseStamped tcpPoseToEefPose(
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

}  // namespace

class PickPlaceMTCNode : public rclcpp::Node
{
public:
  explicit PickPlaceMTCNode(const rclcpp::NodeOptions& options)
  : Node("ur3e_pick_place_mtc", options)
  {
    auto declare = [this](const std::string& name, const auto& default_value, const std::string& desc) {
      rcl_interfaces::msg::ParameterDescriptor d;
      d.description = desc;
      if (!this->has_parameter(name))
        this->declare_parameter(name, default_value, d);
    };

    declare("execute", true, "Execute the best planned solution");
    declare("max_solutions", 1, "Max solutions to plan");

    declare("world_frame", std::string("base_link"), "World frame");
    declare("arm_group", std::string("ur_arm"), "Arm planning group name");
    declare("eef_frame", std::string("robotiq_base_link"), "EEF frame used as IK frame");
    declare("tcp_offset_z", 0.244, "TCP offset along +Z of eef_frame (meters)");

    // Gripper planning (SRDF group: gripper, states: open/closed)
    declare("gripper_group", std::string("gripper"), "Gripper planning group name (SRDF)");
    declare("gripper_open_state", std::string("open"), "Named SRDF group state for open");
    declare("gripper_closed_state", std::string("closed"), "Named SRDF group state for closed");

    // Objects
    declare("cube_id", std::string("pick_cube"), "Cube collision object id");
    declare("cube_x", 0.25, "Cube x in world");
    declare("cube_y", 0.5, "Cube y in world");
    declare("cube_z", 0.05, "Cube ground z (bottom contact) in world");
    declare("cube_size_x", 0.04, "Cube size x");
    declare("cube_size_y", 0.04, "Cube size y");
    declare("cube_size_z", 0.04, "Cube size z");

    declare("bin_id", std::string("place_bin"), "Bin collision object id");
    declare("bin_x", -0.50, "Bin x in world");
    declare("bin_y", 0.40, "Bin y in world");
    declare("bin_z", 0.05, "Bin ground z (bottom contact) in world");
    declare("bin_size_x", 0.20, "Bin size x");
    declare("bin_size_y", 0.20, "Bin size y");
    declare("bin_size_z", 0.10, "Bin size z");

    // Target TCP heights
    declare("pregrasp_z", 0.20, "Pregrasp TCP z in world");
    declare("preplace_z", 0.25, "Preplace TCP z in world");

    // Cartesian solver settings
    declare("cart_step_size", 0.002, "Cartesian step size (m)");
    declare("cart_vel_scale", 0.2, "Cartesian velocity scale");
    declare("cart_acc_scale", 0.2, "Cartesian acceleration scale");

    // Distances
    declare("approach_min", 0.01, "Approach min distance");
    declare("approach_max", 0.12, "Approach max distance");
    declare("lift_min", 0.10, "Lift min distance");
    declare("lift_max", 0.18, "Lift max distance");
    declare("lower_min", 0.01, "Lower min distance");
    declare("lower_max", 0.16, "Lower max distance");
    declare("retreat_min", 0.10, "Retreat min distance");
    declare("retreat_max", 0.18, "Retreat max distance");

    // Directions
    declare("approach_dir_z", -1.0, "Approach direction along eef_frame Z (+1 or -1)");
    declare("retreat_dir_z", 1.0, "Retreat direction along eef_frame Z (+1 or -1)");

    // Wait for valid joint state
    declare("state_timeout", 30.0, "Seconds to wait for valid robot state");
  }

  void run()
  {
    RCLCPP_INFO(this->get_logger(), "PickPlaceMTC starting...");

    if (!waitForValidState())
      return;

    setupScene();

    auto task = createTask();

    RCLCPP_INFO(this->get_logger(), "Planning...");
    try {
      task.init();
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Task init failed: %s", e.what());
      return;
    }

    const int max_solutions = this->get_parameter("max_solutions").as_int();
    if (!task.plan(max_solutions)) {
      RCLCPP_ERROR(this->get_logger(), "Planning failed.");
      return;
    }

    const bool execute = this->get_parameter("execute").as_bool();
    if (execute) {
      RCLCPP_INFO(this->get_logger(), "Executing...");
      if (!task.execute(*task.solutions().front())) {
        RCLCPP_ERROR(this->get_logger(), "Execution failed.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Execution done.");
    } else {
      RCLCPP_INFO(this->get_logger(), "execute:=false, skipping execution.");
    }
  }

private:
  bool waitForValidState()
  {
    const double timeout = this->get_parameter("state_timeout").as_double();
    RCLCPP_INFO(this->get_logger(), "Waiting for valid robot state (timeout %.1fs)...", timeout);

    const auto start = this->now();

    static auto psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      shared_from_this(), "robot_description");

    static bool monitor_started = false;
    if (!monitor_started) {
      psm->startStateMonitor("/joint_states");
      psm->startSceneMonitor();
      monitor_started = true;
      RCLCPP_INFO(this->get_logger(), "Started planning scene monitor");
    }

    while (rclcpp::ok() && (this->now() - start).seconds() < timeout) {
      rclcpp::spin_some(this->get_node_base_interface());

      auto current_state = psm->getStateMonitor()->getCurrentState();
      if (current_state) {
        auto stamp = psm->getStateMonitor()->getCurrentStateTime();
        if (stamp.seconds() > 0.0) {
          RCLCPP_INFO(this->get_logger(), "✓ Received valid state at t=%.3f", stamp.seconds());
          return true;
        }
      }

      rclcpp::sleep_for(std::chrono::milliseconds(200));
    }

    RCLCPP_ERROR(this->get_logger(), "Timeout! Never received valid robot state");
    return false;
  }

  void setupScene()
  {
    RCLCPP_INFO(this->get_logger(), "Setting up planning scene...");

    moveit::planning_interface::PlanningSceneInterface psi;

    const auto world_frame = this->get_parameter("world_frame").as_string();
    const auto cube_id = this->get_parameter("cube_id").as_string();
    const auto bin_id  = this->get_parameter("bin_id").as_string();

    const double cube_x = this->get_parameter("cube_x").as_double();
    const double cube_y = this->get_parameter("cube_y").as_double();
    const double cube_z = this->get_parameter("cube_z").as_double();
    const double cube_sx = this->get_parameter("cube_size_x").as_double();
    const double cube_sy = this->get_parameter("cube_size_y").as_double();
    const double cube_sz = this->get_parameter("cube_size_z").as_double();

    const double bin_x = this->get_parameter("bin_x").as_double();
    const double bin_y = this->get_parameter("bin_y").as_double();
    const double bin_z = this->get_parameter("bin_z").as_double();
    const double bin_sx = this->get_parameter("bin_size_x").as_double();
    const double bin_sy = this->get_parameter("bin_size_y").as_double();
    const double bin_sz = this->get_parameter("bin_size_z").as_double();

    std::vector<moveit_msgs::msg::CollisionObject> objs;
    objs.push_back(makeBoxOnGround(cube_id, world_frame, cube_x, cube_y, cube_z, cube_sx, cube_sy, cube_sz));
    objs.push_back(makeBoxOnGround(bin_id,  world_frame, bin_x,  bin_y,  bin_z,  bin_sx,  bin_sy,  bin_sz));

    psi.applyCollisionObjects(objs);

    RCLCPP_INFO(this->get_logger(), "  Added cube '%s' at [%.2f, %.2f, %.2f] in '%s'",
                cube_id.c_str(), cube_x, cube_y, cube_z, world_frame.c_str());
    RCLCPP_INFO(this->get_logger(), "  Added bin '%s' at [%.2f, %.2f, %.2f] in '%s'",
                bin_id.c_str(), bin_x, bin_y, bin_z, world_frame.c_str());

    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

  mtc::Task createTask()
  {
    const auto world_frame = this->get_parameter("world_frame").as_string();
    const auto arm_group   = this->get_parameter("arm_group").as_string();
    const auto eef_frame   = this->get_parameter("eef_frame").as_string();
    const double tcp_offset_z = this->get_parameter("tcp_offset_z").as_double();

    const auto gripper_group = this->get_parameter("gripper_group").as_string();
    const auto gripper_open_state = this->get_parameter("gripper_open_state").as_string();
    const auto gripper_closed_state = this->get_parameter("gripper_closed_state").as_string();

    RCLCPP_INFO(this->get_logger(), "Creating MTC task:");
    RCLCPP_INFO(this->get_logger(), "  world_frame: %s", world_frame.c_str());
    RCLCPP_INFO(this->get_logger(), "  arm_group: %s", arm_group.c_str());
    RCLCPP_INFO(this->get_logger(), "  gripper_group: %s", gripper_group.c_str());
    RCLCPP_INFO(this->get_logger(), "  eef_frame: %s", eef_frame.c_str());
    RCLCPP_INFO(this->get_logger(), "  tcp_offset_z: %.3f m", tcp_offset_z);

    auto pipeline = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this(), "ompl");

    auto cart = std::make_shared<mtc::solvers::CartesianPath>();
    cart->setStepSize(this->get_parameter("cart_step_size").as_double());
    cart->setMaxVelocityScalingFactor(this->get_parameter("cart_vel_scale").as_double());
    cart->setMaxAccelerationScalingFactor(this->get_parameter("cart_acc_scale").as_double());

    auto joint_interp = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    const auto cube_id = this->get_parameter("cube_id").as_string();
    const double cube_x = this->get_parameter("cube_x").as_double();
    const double cube_y = this->get_parameter("cube_y").as_double();
    const double pregrasp_z = this->get_parameter("pregrasp_z").as_double();

    const double bin_x = this->get_parameter("bin_x").as_double();
    const double bin_y = this->get_parameter("bin_y").as_double();
    const double preplace_z = this->get_parameter("preplace_z").as_double();

    const double approach_min = this->get_parameter("approach_min").as_double();
    const double approach_max = this->get_parameter("approach_max").as_double();
    const double lift_min = this->get_parameter("lift_min").as_double();
    const double lift_max = this->get_parameter("lift_max").as_double();
    const double lower_min = this->get_parameter("lower_min").as_double();
    const double lower_max = this->get_parameter("lower_max").as_double();
    const double retreat_min = this->get_parameter("retreat_min").as_double();
    const double retreat_max = this->get_parameter("retreat_max").as_double();

    const double approach_dir_z = this->get_parameter("approach_dir_z").as_double();
    const double retreat_dir_z  = this->get_parameter("retreat_dir_z").as_double();

    mtc::Task task;
    task.stages()->setName("pick_place");
    task.loadRobotModel(shared_from_this(), "robot_description");

    // MTC global properties
    task.setProperty("group", arm_group);
    task.setProperty("eef", gripper_group);
    task.setProperty("ik_frame", eef_frame);

    geometry_msgs::msg::Quaternion q_tcp;
    q_tcp.w = 1.0;

    const auto pregrasp_tcp = makePoseKeepOrientation(world_frame, q_tcp, cube_x, cube_y, pregrasp_z);
    const auto preplace_tcp = makePoseKeepOrientation(world_frame, q_tcp, bin_x,  bin_y,  preplace_z);

    const auto pregrasp_eef = tcpPoseToEefPose(pregrasp_tcp, tcp_offset_z);
    const auto preplace_eef = tcpPoseToEefPose(preplace_tcp, tcp_offset_z);

    task.add(std::make_unique<mtc::stages::CurrentState>("current"));

    // Open gripper before approaching
    {
      auto s = std::make_unique<mtc::stages::MoveTo>("open_gripper", joint_interp);
      s->setGroup(gripper_group);
      s->setGoal(gripper_open_state);
      task.add(std::move(s));
    }

    // Move arm to pregrasp
    {
      auto s = std::make_unique<mtc::stages::MoveTo>("move_to_pregrasp", pipeline);
      s->setGroup(arm_group);
      s->setIKFrame(eef_frame);
      s->setGoal(pregrasp_eef);
      task.add(std::move(s));
    }

    // Approach
    {
      auto s = std::make_unique<mtc::stages::MoveRelative>("approach", cart);
      s->setGroup(arm_group);
      s->setIKFrame(eef_frame);
      s->setMinMaxDistance(approach_min, approach_max);

      geometry_msgs::msg::Vector3Stamped dir;
      dir.header.frame_id = eef_frame;
      dir.vector.z = approach_dir_z;
      s->setDirection(dir);

      task.add(std::move(s));
    }

    // Close gripper (planned)
    {
      auto s = std::make_unique<mtc::stages::MoveTo>("close_gripper", joint_interp);
      s->setGroup(gripper_group);
      s->setGoal(gripper_closed_state);
      task.add(std::move(s));
    }

    // Attach after closing
    {
      auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("attach_cube");
      s->attachObject(cube_id, eef_frame);
      task.add(std::move(s));
    }

    // Lift
    {
      auto s = std::make_unique<mtc::stages::MoveRelative>("lift", cart);
      s->setGroup(arm_group);
      s->setIKFrame(eef_frame);
      s->setMinMaxDistance(lift_min, lift_max);

      geometry_msgs::msg::Vector3Stamped dir;
      dir.header.frame_id = world_frame;
      dir.vector.z = 1.0;
      s->setDirection(dir);

      task.add(std::move(s));
    }

    // Move to preplace
    {
      auto s = std::make_unique<mtc::stages::MoveTo>("move_to_preplace", pipeline);
      s->setGroup(arm_group);
      s->setIKFrame(eef_frame);
      s->setGoal(preplace_eef);
      task.add(std::move(s));
    }

    // Lower
    {
      auto s = std::make_unique<mtc::stages::MoveRelative>("lower", cart);
      s->setGroup(arm_group);
      s->setIKFrame(eef_frame);
      s->setMinMaxDistance(lower_min, lower_max);

      geometry_msgs::msg::Vector3Stamped dir;
      dir.header.frame_id = world_frame;
      dir.vector.z = -1.0;
      s->setDirection(dir);

      task.add(std::move(s));
    }

    // Open gripper (planned)
    {
      auto s = std::make_unique<mtc::stages::MoveTo>("open_gripper_release", joint_interp);
      s->setGroup(gripper_group);
      s->setGoal(gripper_open_state);
      task.add(std::move(s));
    }

    // Detach after opening
    {
      auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("detach_cube");
      s->detachObject(cube_id, eef_frame);
      task.add(std::move(s));
    }

    // Retreat
    {
      auto s = std::make_unique<mtc::stages::MoveRelative>("retreat", cart);
      s->setGroup(arm_group);
      s->setIKFrame(eef_frame);
      s->setMinMaxDistance(retreat_min, retreat_max);

      geometry_msgs::msg::Vector3Stamped dir;
      dir.header.frame_id = eef_frame;
      dir.vector.z = retreat_dir_z;
      s->setDirection(dir);

      task.add(std::move(s));
    }

    return task;
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<PickPlaceMTCNode>(options);

  // rclcpp::executors::MultiThreadedExecutor exec;
  // exec.add_node(node);

  // rclcpp::sleep_for(std::chrono::seconds(1));
  
  node->run();
  
  // auto spin_thread = std::thread([&exec]() { exec.spin(); });

  // spin_thread.join();
  // exec.cancel();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
