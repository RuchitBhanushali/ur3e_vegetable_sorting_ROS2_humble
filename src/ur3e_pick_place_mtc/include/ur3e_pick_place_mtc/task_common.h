#pragma once
#include <string>

namespace ur3e_pick_place_mtc {

struct PickPlaceParams {
  // frames/groups
  std::string arm_group{"ur_arm"};
  std::string eef_frame{"tool0"};
  std::string world_frame{"world"};

  // TCP offset along +Z of eef_frame (meters).
  // If the actual TCP is in front of eef_frame, set this (e.g. 0.244).
  double tcp_offset_z{0.0};

  // scene numbers (meters) - treated as GROUND height for Z
  double cube_x{0.0};
  double cube_y{0.0};
  double cube_z{0.0};

  double bin_x{0.0};
  double bin_y{0.0};
  double bin_z{0.0};

  // absolute TCP Z targets in world
  double pregrasp_z{0.0};
  double preplace_z{0.0};
};

}  // namespace ur3e_pick_place_mtc
