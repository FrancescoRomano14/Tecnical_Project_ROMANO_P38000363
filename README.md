# Multi-Robot ArUco Navigation

Multi-robot coordination system for autonomous navigation using ArUco marker detection. A fixed manipulator (KUKA IIWA) scans the environment exposing visual markers, while a mobile robot (Fra2mo) navigates autonomously to target sectors.

## Repository Content
- **multirobot_control** : Custom ROS2 package 
- **multirobot_coordinator**: Custom ROS2 package 
- **ros2_fra2mo**: Modified mobile robot package (custom map + optimized Nav2)

## System Components

**Custom Nodes:**
- `iiwa_scanner_node`  - Continuous oscillation control (±170°)
- `task_allocator_node`  - ArUco ID → Sector mapping {5:'N', 10:'E', 15:'S', 20:'W'}
- `fra2mo_executor_node`  - Nav2 action client with debounce

**Communication:**
- 8 topics (1 custom: `/task/sector`)
- 1 action (`navigate_to_pose`)

## Dependencies

- ROS2 Humble
- aruco_ros (PAL Robotics)
- iiwa_ros2 (ICube Laboratory)
- Nav2 stack
- Gazebo Ignition Fortress

## Modifications to ros2_fra2mo

**Custom additions:**
- `maps/arena_map_rotated.pgm/yaml` - SLAM-generated map (0.04 m/pixel)

**Optimized parameters** (`config/navigation.yaml`):
- `max_particles: 200` (was 500) → -60% AMCL load
- `xy_goal_tolerance: 0.25` (was 0.1) → +150% robustness


## Author

Francesco - Exam Project
