#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <linkattacher_msgs/srv/attach_link.hpp>
#include <linkattacher_msgs/srv/detach_link.hpp>

const double tau = 2 * M_PI;

class PickAndPlace
{
public:
    PickAndPlace(rclcpp::Node::SharedPtr node)
        : move_group(node, "ur_arm"), //
          gripper(node, "gripper"), //gripper
          planning_scene_interface(),
          logger(rclcpp::get_logger("PickAndPlace")),

          // link attacher
          node_(node)
    {
        // All poses below are expressed in the Gazebo world frame
        // (see pick_place_table_world.sdf).
        move_group.setPoseReferenceFrame("base_link");

        attach_client = node_->create_client<linkattacher_msgs::srv::AttachLink>("/ATTACHLINK");
        detach_client = node_->create_client<linkattacher_msgs::srv::DetachLink>("/DETACHLINK");

        // Add static collision geometry that matches pick_place_table_world.sdf
        // add_collision_objects_from_world();
    }

    // Collision geometry based on pick_place_table_world.sdf
    // - cube (pick object) at: 0.35 -0.2 0.60 (size 0.04)
    // - bin at: 0.60 0.15 0.55 (base + 4 walls)
    void add_collision_objects_from_world()
    {
        std::vector<moveit_msgs::msg::CollisionObject> objs;
        objs.reserve(7);

        // // -------- Ground plane (thin box) --------
        // {
        //     moveit_msgs::msg::CollisionObject co;
        //     co.id = "ground_plane";
        //     co.header.frame_id = "world";

        //     shape_msgs::msg::SolidPrimitive prim;
        //     prim.type = prim.BOX;
        //     prim.dimensions = {5.0, 5.0, 0.001}; // 5x5m, 1mm thick

        //     geometry_msgs::msg::Pose p;
        //     p.orientation.w = 1.0;
        //     p.position.x = 0.0;
        //     p.position.y = 0.0;
        //     p.position.z = 0.0; // top surface at z=0

        //     co.primitives.push_back(prim);
        //     co.primitive_poses.push_back(p);
        //     co.operation = co.ADD;
        //     objs.push_back(co);
        // }

        // -------- pickup_table (thin box) --------
        {
            moveit_msgs::msg::CollisionObject co;
            co.id = "pickup_table";
            co.header.frame_id = "base_link";

            shape_msgs::msg::SolidPrimitive prim;
            prim.type = prim.BOX;
            prim.dimensions = {0.79, 0.59, 0.001}; // 5x5m, 1mm thick

            geometry_msgs::msg::Pose p;
            p.orientation.w = 1.0;
            p.position.x = 0.0;
            p.position.y = 0.555;  // center - edge at 0.37
            p.position.z = 0.0859; // top surface at z=0

            co.primitives.push_back(prim);
            co.primitive_poses.push_back(p);
            co.operation = co.ADD;
            objs.push_back(co);
        }

        // -------- side wall 1 (thin box) --------

        {
            moveit_msgs::msg::CollisionObject co;
            co.id = "wall_1";
            co.header.frame_id = "base_link";

            shape_msgs::msg::SolidPrimitive prim;
            prim.type = prim.BOX;
            prim.dimensions = {0.01, 0.9, 0.90}; // 5x5m, 1mm thick

            geometry_msgs::msg::Pose p;
            p.orientation.w = 1.0;
            p.position.x = 0.45;
            p.position.y = 0.0;  // center - edge at 0.37
            p.position.z = 0.45; // top surface at z=0

            co.primitives.push_back(prim);
            co.primitive_poses.push_back(p);
            co.operation = co.ADD;
            objs.push_back(co);
        }

        {
            moveit_msgs::msg::CollisionObject co;
            co.id = "wall_2";
            co.header.frame_id = "base_link";

            shape_msgs::msg::SolidPrimitive prim;
            prim.type = prim.BOX;
            prim.dimensions = {0.01, 0.9, 0.90}; // 5x5m, 1mm thick

            geometry_msgs::msg::Pose p;
            p.orientation.w = 1.0;
            p.position.x = -0.45;
            p.position.y = 0.0;  // center - edge at 0.37
            p.position.z = 0.45; // top surface at z=0

            co.primitives.push_back(prim);
            co.primitive_poses.push_back(p);
            co.operation = co.ADD;
            objs.push_back(co);
        }

        {
            moveit_msgs::msg::CollisionObject co;
            co.id = "wall_3";
            co.header.frame_id = "base_link";

            shape_msgs::msg::SolidPrimitive prim;
            prim.type = prim.BOX;
            prim.dimensions = {0.9, 0.01, 0.90}; // 5x5m, 1mm thick

            geometry_msgs::msg::Pose p;
            p.orientation.w = 1.0;
            p.position.x = 0.0;
            p.position.y = -0.45;  // center - edge at 0.37
            p.position.z = 0.45; // top surface at z=0

            co.primitives.push_back(prim);
            co.primitive_poses.push_back(p);
            co.operation = co.ADD;
            objs.push_back(co);
        }

        // // -------- Bin (base + 4 walls) --------
        // const double bin_x = -0.40;
        // const double bin_y = 0.2;
        // const double bin_z = 0;
        // const double wall_thickness = 0.02;

        // // Base: <size>0.20 0.20 0.01</size>, pose 0 0 0.005
        // {
        //     moveit_msgs::msg::CollisionObject co;
        //     co.id = "bin_base";
        //     co.header.frame_id = "base_link";

        //     shape_msgs::msg::SolidPrimitive prim;
        //     prim.type = prim.BOX;
        //     prim.dimensions = {0.20, 0.20, 0.01};

        //     geometry_msgs::msg::Pose p;
        //     p.orientation.w = 1.0;
        //     p.position.x = bin_x + 0.0;
        //     p.position.y = bin_y + 0.0;
        //     p.position.z = bin_z + 0.005;

        //     co.primitives.push_back(prim);
        //     co.primitive_poses.push_back(p);
        //     co.operation = co.ADD;
        //     objs.push_back(co);
        // }

        // // Walls: size 0.20 0.02 0.10, poses in bin link:
        // //  wall_px:  0.10  0.0  0.05
        // //  wall_nx: -0.10  0.0  0.05
        // //  wall_py:  0.0   0.10 0.05
        // //  wall_ny:  0.0  -0.10 0.05
        // auto add_wall = [&](const std::string& id, double lx, double ly, double lz, double sx, double sy, double sz) {
        //     moveit_msgs::msg::CollisionObject co;
        //     co.id = id;
        //     co.header.frame_id = "base_link";

        //     shape_msgs::msg::SolidPrimitive prim;
        //     prim.type = prim.BOX;
        //     prim.dimensions = {sx, sy, sz};

        //     geometry_msgs::msg::Pose p;
        //     p.orientation.w = 1.0;
        //     p.position.x = bin_x + lx;
        //     p.position.y = bin_y + ly;
        //     p.position.z = bin_z + lz;

        //     co.primitives.push_back(prim);
        //     co.primitive_poses.push_back(p);
        //     co.operation = co.ADD;
        //     objs.push_back(co);
        // };

        // // wall_px / wall_nx
        // add_wall("bin_wall_px", 0.10, 0.0, 0.05, wall_thickness, 0.20, 0.10);
        // add_wall("bin_wall_nx", -0.10, 0.0, 0.05, wall_thickness, 0.20, 0.10);
        // // wall_py / wall_ny (note swapped dims)
        // add_wall("bin_wall_py", 0.0, 0.10, 0.05, 0.20, wall_thickness, 0.10);
        // add_wall("bin_wall_ny", 0.0, -0.10, 0.05, 0.20, wall_thickness, 0.10);

        // // -------- Cube (pick object) --------
        // {
        //     moveit_msgs::msg::CollisionObject co;
        //     co.id = "cube";
        //     co.header.frame_id = "base_link";

        //     shape_msgs::msg::SolidPrimitive prim;
        //     prim.type = prim.BOX;
        //     prim.dimensions = {0.04, 0.04, 0.04};

        //     geometry_msgs::msg::Pose p;
        //     p.orientation.w = 1.0;
        //     p.position.x = -0.13;
        //     p.position.y = 0.375;
        //     p.position.z = 0.1060;
        //     co.primitives.push_back(prim);
        //     co.primitive_poses.push_back(p);
        //     co.operation = co.ADD;
        //     objs.push_back(co);
        // }

        planning_scene_interface.applyCollisionObjects(objs);
        RCLCPP_INFO(logger, "Added %zu collision objects from world SDF", objs.size());
    }

    void close_gripper()
    {
        gripper.setJointValueTarget("finger_joint", 0.5);
        gripper.move();
        RCLCPP_INFO(logger, "gripper closed execution completed.");
    }

    void open_gripper()
    {
        gripper.setJointValueTarget("finger_joint", 0.0);
        gripper.move();
    }

    void home()
    {
        move_group.setMaxVelocityScalingFactor(1);
        move_group.setMaxAccelerationScalingFactor(1);
        move_group.setPlanningTime(10.0);
        move_group.allowReplanning(false);
        move_group.setGoalTolerance(0.03);
        move_group.setStartStateToCurrentState();

        // --- Pick pose (from pick_place_table_world.sdf) ---
        // cube pose:  x=0.35  y=-0.20  z=0.60
        // Use a small approach offset in +Z so the gripper comes from above.
        geometry_msgs::msg::Pose pick_pose;
        tf2::Quaternion orientation;
        orientation.setRPY(-1.571, 0.002, -0.000);
        pick_pose.orientation = tf2::toMsg(orientation);
        pick_pose.position.x = 0.00;
        pick_pose.position.y = 0.223;
        pick_pose.position.z = 0.694; // approach above the cube


        move_group.setPoseTarget(pick_pose, "wrist_3_link");

        // Planning
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(logger, "Visualizing pick plan: %s", success ? "SUCCESS" : "FAILED");

        // Execution
        if (success)
        {
            move_group.move();
            RCLCPP_INFO(logger, "home motion execution completed.");

        }
        else
        {
            RCLCPP_ERROR(logger, "Motion planning for pick failed!");
        }

    }

    void pick()
    {
        move_group.setMaxVelocityScalingFactor(1);
        move_group.setMaxAccelerationScalingFactor(1);
        move_group.setPlanningTime(10.0);
        move_group.allowReplanning(false);
        move_group.setGoalTolerance(0.03);
        move_group.setStartStateToCurrentState();

        // --- Pick pose (from pick_place_table_world.sdf) ---
        // cube pose:  x=0.35  y=-0.20  z=0.60
        // Use a small approach offset in +Z so the gripper comes from above.
        geometry_msgs::msg::Pose pick_pose;
        tf2::Quaternion orientation;
        orientation.setRPY(-3.124, 0.000, 0.000);
        pick_pose.orientation = tf2::toMsg(orientation);
        pick_pose.position.x = -0.131;
        pick_pose.position.y = 0.370;
        pick_pose.position.z = 0.348; // approach above the cube


        move_group.setPoseTarget(pick_pose, "wrist_3_link");

        // Planning
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(logger, "Visualizing pick plan: %s", success ? "SUCCESS" : "FAILED");

        // Execution
        if (success)
        {
            move_group.move();
            RCLCPP_INFO(logger, "Pick motion execution completed.");

        }
        else
        {
            RCLCPP_ERROR(logger, "Motion planning for pick failed!");
        }
    }

    void place()
    {
        move_group.setMaxVelocityScalingFactor(1);
        move_group.setMaxAccelerationScalingFactor(1);
        move_group.setPlanningTime(10.0);
        move_group.allowReplanning(false);
        move_group.setGoalTolerance(0.03);
        move_group.setStartStateToCurrentState();

        // --- Place pose (from pick_place_table_world.sdf) ---
        // bin pose: x=0.60  y=0.15  z=0.55
        // Put the tool above the bin opening; adjust later for your desired drop height.
        geometry_msgs::msg::Pose place_pose;
        tf2::Quaternion orientation;
        orientation.setRPY(-3.124, 0.000, 0.000);
        place_pose.orientation = tf2::toMsg(orientation);
        place_pose.position.x = -0.133;
        place_pose.position.y = 0.370;
        place_pose.position.z = 0.386;
        move_group.setPoseTarget(place_pose, "wrist_3_link");

        // Planning
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(logger, "Visualizing plan: %s", success ? "SUCCESS" : "FAILED");

        // Execution
        if (success)
        {
            move_group.move();
            RCLCPP_INFO(logger, "Motion execution completed.");
        }
        else
        {
            RCLCPP_ERROR(logger, "Motion planning failed!");
        }
    }

    void attachObject()
    {
        auto request = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();
        request->model1_name = "ur";  // Nome del robot in Gazebo
        request->link1_name = "wrist_3_link"; // Nome del link del gripper
        request->model2_name = "cube_pick"; // Nome dell'oggetto da afferrare
        request->link2_name = "link_1";    // Nome del link dell'oggetto

        while (!attach_client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(logger, "Waiting for the AttachLink service...");
        }

        auto future = attach_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(logger, "Object attached successfully.");
        }
        else
        {
            RCLCPP_ERROR(logger, "Failed to attach object.");
        }
    }

    void detachObject()
    {
        auto request = std::make_shared<linkattacher_msgs::srv::DetachLink::Request>();
        request->model1_name = "ur";  // Nome del robot in Gazebo
        request->link1_name = "wrist_3_link"; // Nome del link del gripper
        request->model2_name = "cube_pick"; // Nome dell'oggetto da afferrare
        request->link2_name = "link_1";    // Nome del link dell'oggetto

        while (!detach_client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(logger, "Waiting for the DetachLink service...");
        }

        auto future = detach_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(logger, "Object detached successfully.");
        }
        else
        {
            RCLCPP_ERROR(logger, "Failed to detach object.");
        }
    }

    void pick_and_place()
    {
        open_gripper();
        rclcpp::sleep_for(std::chrono::seconds(2));

        pick();
        rclcpp::sleep_for(std::chrono::seconds(3));

        close_gripper();
        rclcpp::sleep_for(std::chrono::seconds(2));

        home();
        rclcpp::sleep_for(std::chrono::seconds(3));
        // attachObject();
        // rclcpp::sleep_for(std::chrono::seconds(1));

        place();
        rclcpp::sleep_for(std::chrono::seconds(3));

        open_gripper();
        rclcpp::sleep_for(std::chrono::seconds(2));

        home();
        rclcpp::sleep_for(std::chrono::seconds(3));
        // detachObject();
        // rclcpp::sleep_for(std::chrono::seconds(1));

    }

private:
    moveit::planning_interface::MoveGroupInterface move_group;
    moveit::planning_interface::MoveGroupInterface gripper;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    rclcpp::Client<linkattacher_msgs::srv::AttachLink>::SharedPtr attach_client;
    rclcpp::Client<linkattacher_msgs::srv::DetachLink>::SharedPtr detach_client;
    rclcpp::Logger logger;

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

    rclcpp::Node::SharedPtr node_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("pick_and_place_node");

    PickAndPlace pick_and_place(node);

    // Add Collision object
    pick_and_place.add_collision_objects_from_world();
    rclcpp::sleep_for(std::chrono::seconds(1));

    pick_and_place.pick_and_place();

    rclcpp::shutdown();
    return 0;
}
