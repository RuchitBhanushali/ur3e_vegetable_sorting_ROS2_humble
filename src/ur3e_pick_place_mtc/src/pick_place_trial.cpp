/**
 * @file pick_place_mtc.cpp
 * @brief MoveIt Task Constructor pick & place - NO MoveGroupInterface to avoid conflicts
 */

#include <rclcpp/rclcpp.hpp>
#include <thread>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>

#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

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

    declare("cube_id", std::string("pick_cube"), "Collision object id for pick object");
    declare("bin_id", std::string("place_bin"), "Collision object id for bin");

    declare("cube_x", 0.35, "Cube ground X");
    declare("cube_y", 0.15, "Cube ground Y");
    declare("cube_z", 0.0, "Cube ground Z");
    declare("cube_size_x", 0.04, "Cube size X");
    declare("cube_size_y", 0.04, "Cube size Y");
    declare("cube_size_z", 0.04, "Cube size Z");

    declare("bin_x", 0.35, "Bin ground X");
    declare("bin_y", -0.25, "Bin ground Y");
    declare("bin_z", 0.0, "Bin ground Z");
    declare("bin_size_x", 0.20, "Bin size X");
    declare("bin_size_y", 0.20, "Bin size Y");
    declare("bin_size_z", 0.10, "Bin size Z");

    declare("pregrasp_z", 0.20, "TCP Z for pregrasp above cube");
    declare("preplace_z", 0.25, "TCP Z for preplace above bin");

    declare("cart_step_size", 0.002, "Cartesian step size (m)");
    declare("cart_vel_scale", 0.2, "Cartesian max velocity scaling");
    declare("cart_acc_scale", 0.2, "Cartesian max acceleration scaling");

    declare("approach_min", 0.01, "Approach min distance");
    declare("approach_max", 0.12, "Approach max distance");
    declare("lift_min", 0.10, "Lift min distance");
    declare("lift_max", 0.18, "Lift max distance");
    declare("lower_min", 0.01, "Lower min distance");
    declare("lower_max", 0.16, "Lower max distance");
    declare("retreat_min", 0.10, "Retreat min distance");
    declare("retreat_max", 0.18, "Retreat max distance");

    declare("approach_dir_z", -1.0, "Approach direction Z in eef_frame");
    declare("retreat_dir_z",  1.0, "Retreat direction Z in eef_frame");
    
    RCLCPP_INFO(this->get_logger(), "PickPlaceMTCNode constructed");
  }

  void run()
  {
    RCLCPP_INFO(this->get_logger(), "=== MTC Pick & Place Starting ===");
    
    // Wait for valid joint states
    if (!waitForJointStates(10.0)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to receive joint_states!");
      return;
    }
  
    RCLCPP_INFO(this->get_logger(), "✓ Joint states received. Setting up scene...");
    rclcpp::sleep_for(std::chrono::seconds(1));
  
    setupScene();
    
    RCLCPP_INFO(this->get_logger(), "Creating and planning task...");
    auto task = createTask();

    int max_solutions = this->get_parameter("max_solutions").as_int();
    bool execute = this->get_parameter("execute").as_bool();

    try {
      task.init();
    } catch (const mtc::InitStageException& e) {
      RCLCPP_ERROR(this->get_logger(), "Task init failed: %s", e.what());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Planning with max_solutions=%d...", max_solutions);
    if (!task.plan(max_solutions)) {
      RCLCPP_ERROR(this->get_logger(), "Planning failed!");
      task.printState();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "✓ Planning succeeded! Found %zu solutions.", 
                task.solutions().size());
    task.introspection().publishSolution(*task.solutions().front());

    if (execute) {
      RCLCPP_INFO(this->get_logger(), "Executing solution...");
      auto res = task.execute(*task.solutions().front());
      if (res.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Execution failed: %d", res.val);
      } else {
        RCLCPP_INFO(this->get_logger(), "✓ Execution succeeded!");
      }
    }
  }

private:
  bool waitForJointStates(double timeout)
  {
    RCLCPP_INFO(this->get_logger(), "Waiting for /joint_states (timeout=%.1fs)...", timeout);
    
    bool received = false;
    auto sub = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",
      rclcpp::QoS(10),
      [&received, this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (msg->header.stamp.sec > 0) {
          RCLCPP_INFO(this->get_logger(), 
                      "✓ Received joint_states at time %.3f", 
                      msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);
          received = true;
        }
      }
    );

    auto start = this->now();
    while (rclcpp::ok() && !received) {
      if ((this->now() - start).seconds() > timeout) {
        RCLCPP_ERROR(this->get_logger(), "Timeout waiting for joint_states");
        return false;
      }
      rclcpp::spin_some(this->get_node_base_interface());
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    return received;
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
    objs.push_back(makeBoxOnGround(cube_id, world_frame, cube_x, cube_y, cube_z, 
                                   cube_sx, cube_sy, cube_sz));
    objs.push_back(makeBoxOnGround(bin_id, world_frame, bin_x, bin_y, bin_z, 
                                   bin_sx, bin_sy, bin_sz));

    psi.applyCollisionObjects(objs);
    
    RCLCPP_INFO(this->get_logger(), "  ✓ Added cube at [%.2f, %.2f, %.2f]", 
                cube_x, cube_y, cube_z);
    RCLCPP_INFO(this->get_logger(), "  ✓ Added bin at [%.2f, %.2f, %.2f]", 
                bin_x, bin_y, bin_z);
    
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

  mtc::Task createTask()
  {
    const auto world_frame = this->get_parameter("world_frame").as_string();
    const auto arm_group   = this->get_parameter("arm_group").as_string();
    const auto eef_frame   = this->get_parameter("eef_frame").as_string();
    const double tcp_offset_z = this->get_parameter("tcp_offset_z").as_double();

    RCLCPP_INFO(this->get_logger(), "Task config: world=%s, group=%s, eef=%s, tcp_z=%.3f",
                world_frame.c_str(), arm_group.c_str(), eef_frame.c_str(), tcp_offset_z);
    
    auto pipeline = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this(), "ompl");

    auto cart = std::make_shared<mtc::solvers::CartesianPath>();
    cart->setStepSize(this->get_parameter("cart_step_size").as_double());
    cart->setMaxVelocityScalingFactor(this->get_parameter("cart_vel_scale").as_double());
    cart->setMaxAccelerationScalingFactor(this->get_parameter("cart_acc_scale").as_double());

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

    task.setProperty("group", arm_group);
    task.setProperty("ik_frame", eef_frame);

    geometry_msgs::msg::Quaternion q_tcp;
    q_tcp.w = 1.0;

    const auto pregrasp_tcp = makePoseKeepOrientation(world_frame, q_tcp, cube_x, cube_y, pregrasp_z);
    const auto preplace_tcp = makePoseKeepOrientation(world_frame, q_tcp, bin_x, bin_y, preplace_z);

    const auto pregrasp_eef = tcpPoseToEefPose(pregrasp_tcp, tcp_offset_z);
    const auto preplace_eef = tcpPoseToEefPose(preplace_tcp, tcp_offset_z);

    // Stage 1: Current state
    task.add(std::make_unique<mtc::stages::CurrentState>("current"));

    // Stage 2: Move to pregrasp
    {
      auto s = std::make_unique<mtc::stages::MoveTo>("move_to_pregrasp", pipeline);
      s->setGroup(arm_group);
      s->setIKFrame(eef_frame);
      s->setGoal(pregrasp_eef);
      task.add(std::move(s));
    }

    // Stage 3: Approach
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

    // Stage 4: Attach cube
    {
      auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("attach_cube");
      s->attachObject(cube_id, eef_frame);
      task.add(std::move(s));
    }

    // Stage 5: Lift
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

    // Stage 6: Move to preplace
    {
      auto s = std::make_unique<mtc::stages::MoveTo>("move_to_preplace", pipeline);
      s->setGroup(arm_group);
      s->setIKFrame(eef_frame);
      s->setGoal(preplace_eef);
      task.add(std::move(s));
    }

    // Stage 7: Lower
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

    // Stage 8: Detach cube
    {
      auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("detach_cube");
      s->detachObject(cube_id, eef_frame);
      task.add(std::move(s));
    }

    // Stage 9: Retreat
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

// CRITICAL: Single executor, simple main
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<PickPlaceMTCNode>(options);

  // Single-threaded executor (avoids conflicts with move_group)
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Spin in background thread
  auto spin_thread = std::thread([&executor]() {
    executor.spin();
  });

  // Run the task
  node->run();

  // Cleanup
  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();

  return 0;
}