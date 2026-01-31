# UR3e Vegetable Sorting System (ROS 2 Humble)

This repository contains a **ROS 2 Humble–based robotic manipulation project** for **vegetable sorting using a UR3e robotic arm**, combining **perception, motion planning, and simulation**.

The codebase is structured so that **perception and manipulation can be run independently or together**, making development, debugging, and extension straightforward.

---

## What This Repository Contains

- **UR3e manipulation stack**
  - MoveIt 2 motion planning
  - Gazebo-based simulation
  - Gripper control and execution
- **Vegetable perception module**
  - YOLOv8-based detection
  - Depth-based 3D position estimation using Intel RealSense
- **Docker-based runtime**
  - Reproducible environment
  - No manual dependency management

---

## Code Organization

- `src/ur3e_robotiq_gz_sim/`  
  Main manipulation and simulation package (UR3e + Robotiq gripper + MoveIt + Gazebo).

- `src/ros_vegetable_detection/`  
  Perception module (Git submodule).  
  Handles vegetable detection, depth processing, and publishes object information.

- `third_party.repos`  
  External ROS dependencies (UR, Robotiq, etc.), fetched automatically during build.

- `docker/` and `compose.yaml`  
  Docker setup used to build and run the complete system.

---

## Quick Start (Recommended: Docker)

### Prerequisites

- Linux (tested on Ubuntu 22.04)
- Docker & Docker Compose
- X11 display (for RViz / Gazebo)

### 1. Clone the repository (with submodule)
```bash
git clone --recurse-submodules https://github.com/RuchitBhanushali/ur3e_vegetable_sorting_ROS2_humble.git
cd ur3e_vegetable_sorting_ROS2_humble
```

If already cloned:
```bash
git submodule update --init --recursive
```

### 2. Allow GUI access (host)
```bash
xhost +local:docker
```

### 3. Build and run
```bash
docker compose build
docker compose up
```

You now have a shell inside a ROS 2 Humble environment with all packages built.

## Local Build (Without Docker)
### Prerequisites

- Linux (tested on Ubuntu 22.04)
- ROS2 Humble installed

```bash
mkdir -p ws/src
cd ws/src
git clone --recurse-submodules https://github.com/RuchitBhanushali/ur3e_vegetable_sorting_ROS2_humble.git
cd ..
vcs import src < src/ur3e_vegetable_sorting_ROS2_humble/third_party.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```


## Manipulation / Simulation Workflow

### Source the workspace

Inside the container:

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
```

View UR3e MoveIt

```bash
ros2 launch ur3e_robotiq_gz_sim View_ur_robotiq_camera.launch.py
```

Run Pick and Place with MTC - MoveIt + Gazebo - (working partialy)

Terminal 1
```bash
ros2 launch ur3e_robotiq_gz_sim gz_spwan_ur3e_robotiq_camera.launch.py
```

Terminal 2
```bash
ros2 launch ur3e_pick_place_mtc pick_place_mtc.launch.py
```

This will:
- Spawn the UR3e robot in Gazebo
- Start MTC in rviz
- Allow motion planning and execution


Run Pick and Place with movegroupInterface - MoveIt + Gazebo

```bash
ros2 launch ur3e_robotiq_gz_sim gz_moveit_pick_place.launch.py
```

This will:
- Spawn the UR3e robot in Gazebo
- Start MoveIt 2
- Allow motion planning and execution via RViz


## Perception Workflow (Vegetable Detection)

The perception module can be tested independently or integrated with manipulation.

### A) Standalone perception test (no ROS, quick check)

Used to validate the YOLO model and camera setup.

```bash
yolo predict \
  model=./runs/detect/vegetable_train/exp_final/weights/best.pt \
  source=<camera_id> \
  show=True
```

Replace <camera_id> with your camera index (e.g. 0, 1).

### B) Standalone depth + detection script

Runs vegetable detection and computes 3D coordinates using RealSense depth.

```bash
cd src/ros_vegetable_detection
python scripts/depth_display.py
```

This script:
- Detects vegetables
- Computes 3D positions from the point cloud
- Optionally visualizes results

### C) Perception as a ROS 2 node

Run the perception node that publishes detection results to ROS topics.
```bash
ros2 run yolo_realsense yolo_node
```

Check published topics:
```bash
ros2 topic list
```

Inspect detection output:
```bash
ros2 topic echo <topic_name>
```

These topics are intended to be consumed by the manipulation pipeline for grasp planning.


### Typical Development Flow

1. Test perception alone
    - Verify detection and depth output.

2. Run manipulation in simulation
    - Test motion planning and gripper control.

3. Integrate perception + manipulation
    - Use detected object poses as grasp targets.

This separation makes debugging significantly easier.


### Dependency Handling

- No third-party ROS packages are committed directly.
- External dependencies are listed in third_party.repos.
- System dependencies are resolved via rosdep.
- Docker builds everything from scratch.
This keeps the repository clean and reproducible.

### Collaboration

- Manipulation, simulation, MoveIt 2, Docker integration
    Ruchit Bhanushali

- Vegetable detection & depth perception
    Kaung Sett Thu
    (included as a Git submodule)

### Notes

- The system currently runs fully in simulation.
- The structure is designed so that real UR3e hardware can be integrated later with minimal changes.
- Perception and manipulation can be developed and tested independently.


### Author

Ruchit Bhanushali
Robotics & Automation / Intelligent Robotics