# Multi-Robot ArUco Navigation

Multi-robot coordination system for autonomous navigation using ArUco marker detection. A fixed manipulator (KUKA IIWA) scans the environment exposing visual markers, while a mobile robot (Fra2mo) navigates autonomously to target sectors.

## Repository Content

- **multirobot_coordinator**: Custom ROS2 package (3 nodes, 459 lines total)
- **ros2_fra2mo**: Modified mobile robot package (custom map + optimized Nav2)

## System Components

**Custom Nodes:**
- `iiwa_scanner_node` (195 lines) - Continuous oscillation control (±170°)
- `task_allocator_node` (105 lines) - ArUco ID → Sector mapping {5:'N', 10:'E', 15:'S', 20:'W'}
- `fra2mo_executor_node` (159 lines) - Nav2 action client with debounce

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
- `rviz_conf/fra2mo_conf.rviz` - Custom visualization config

**Optimized parameters** (`config/navigation.yaml`):
- `max_particles: 200` (was 500) → -60% AMCL load
- `xy_goal_tolerance: 0.25` (was 0.1) → +150% robustness

## Demo

[Video link - Sistema completo]

## Author

Francesco - Exam Project
