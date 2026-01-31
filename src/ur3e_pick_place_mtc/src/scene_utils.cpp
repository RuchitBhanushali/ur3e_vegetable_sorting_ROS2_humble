#include "ur3e_pick_place_mtc/scene_utils.h"

#include <vector>

namespace ur3e_pick_place_mtc {

moveit_msgs::msg::CollisionObject makeBox(
  const std::string& id,
  const std::string& frame_id,
  double x, double y, double z,
  double sx, double sy, double sz)
{
  moveit_msgs::msg::CollisionObject obj;
  obj.id = id;
  obj.header.frame_id = frame_id;

  shape_msgs::msg::SolidPrimitive prim;
  prim.type = shape_msgs::msg::SolidPrimitive::BOX;
  prim.dimensions = {sx, sy, sz};

  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;
  p.orientation.w = 1.0;

  obj.primitives.push_back(prim);
  obj.primitive_poses.push_back(p);
  obj.operation = obj.ADD;
  return obj;
}

void addCubeAndBinOnlyOnGround(
  moveit::planning_interface::PlanningSceneInterface& psi,
  const std::string& world_frame,
  double cube_x, double cube_y, double cube_z_ground,
  double bin_x, double bin_y, double bin_z_ground)
{
  std::vector<moveit_msgs::msg::CollisionObject> objs;

  // Dimensions (must match SDF)
  const double cube_s = 0.04;

  const double bin_sx = 0.2;
  const double bin_sy = 0.2;
  const double wall_t = 0.01;
  const double bin_h  = 0.10;
  const double floor_t = 0.01;

  // Interpret cube_z as GROUND height => cube center = ground + half size
  const double cube_center_z = cube_z_ground + cube_s * 0.5;

  // Bin: floor sits on ground => floor center = ground + floor_t/2
  const double floor_center_z = bin_z_ground + floor_t * 0.5;

  // Walls are height bin_h => wall center = ground + bin_h/2
  const double wall_center_z  = bin_z_ground + bin_h * 0.5;

  // Cube
  objs.push_back(makeBox("pick_cube", world_frame,
                         cube_x, cube_y, cube_center_z,
                         cube_s, cube_s, cube_s));

  // Bin floor
  objs.push_back(makeBox("bin_floor", world_frame,
                         bin_x, bin_y, floor_center_z,
                         bin_sx, bin_sy, floor_t));

  // Bin walls
  // nx wall (left in x)
  objs.push_back(makeBox("bin_wall_nx", world_frame,
                         bin_x - (bin_sx * 0.5 - wall_t * 0.5),
                         bin_y,
                         wall_center_z,
                         wall_t, bin_sy, bin_h));

  // px wall (right in x)
  objs.push_back(makeBox("bin_wall_px", world_frame,
                         bin_x + (bin_sx * 0.5 - wall_t * 0.5),
                         bin_y,
                         wall_center_z,
                         wall_t, bin_sy, bin_h));

  // ny wall (bottom in y)
  objs.push_back(makeBox("bin_wall_ny", world_frame,
                         bin_x,
                         bin_y - (bin_sy * 0.5 - wall_t * 0.5),
                         wall_center_z,
                         bin_sx, wall_t, bin_h));

  // py wall (top in y)
  objs.push_back(makeBox("bin_wall_py", world_frame,
                         bin_x,
                         bin_y + (bin_sy * 0.5 - wall_t * 0.5),
                         wall_center_z,
                         bin_sx, wall_t, bin_h));

  psi.applyCollisionObjects(objs);
}

}  // namespace ur3e_pick_place_mtc
