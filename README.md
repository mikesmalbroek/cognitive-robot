# Cognitive Robot

ROS 2 (Humble) project for the MIRTE Master robot. The robot builds a map of the
environment, autonomously navigates to two stations, reads a digital clock with
OCR at one station, and places rings on an abacus at the other.

It runs both in **Gazebo simulation** and on the **real robot**.

---

## Start here

New to the project? Read the run guide first:

➡️ **[docs/integration.md](docs/integration.md)** — one launch command per phase, for
both Gazebo and the real robot.

### The mission, in three phases
1. **Phase 1 — Mapping:** drive around manually to build a map and register the two stations.
2. **Phase 2 — Autonomous mission:** robot drives to Station A, reads the clock, drives to Station B.
3. **Phase 3 — Manipulation:** the arm places rings on the abacus.

---

## Documentation

| Guide                                                        | What it covers |
|--------------------------------------------------------------|----------------|
| [docs/integration.md](docs/integration.md)                   | How to launch each phase (Gazebo + real robot) |
| [docs/computer_vision.md](docs/computer_vision.md)           | CV services (clock OCR, abacus & station detection) |
| [docs/slam_and_navigation.md](docs/slam_and_navigation.md)   | SLAM mapping and Nav2 navigation |
| [docs/PRESENTATION_SUMMARY.md](docs/PRESENTATION_SUMMARY.md) | High-level summary of the project |

---

## Repository layout

| Folder | Contents |
|--------|----------|
| `cognitive_robot/` | The main ROS 2 package — nodes (CV services, mission, manipulation), [launch files](cognitive_robot/launch/launch_readme.md), and [tests](cognitive_robot/test/test_readme.md). |
| `cognitive_robot_interfaces/` | Custom ROS 2 service definitions (`.srv`). See [its readme](cognitive_robot_interfaces/cognitive_robot_interfaces_readme.md). |
| `config/` | Nav2 / SLAM parameters and the RViz config. See [its readme](config/config_readme.md). |
| `maps/` | Maps and station locations saved during Phase 1. See [its readme](maps/maps_readme.md). |
| `gazebo_map_load/` | Gazebo world objects that mimic the real demo playground. See [its readme](gazebo_map_load/gazebo_map_README.md). |
| `docs/` | Documentation: run guides, command snippets, ArUco markers, robot description. |
| `debug_logs/` | Diagnostic notes from debugging the Phase 2 real-robot runs. See [its readme](debug_logs/debug_logs_readme.md). |
| `testing_only/` | Throwaway development/test scripts — **not** part of the robot pipeline. See [its readme](testing_only/testing_only_readme.md). |

---

## Quick build

```bash
cd ~/mirte_ws
colcon build
source install/setup.bash
```

> After changing a `.srv` file, run the full `colcon build` (not
> `--packages-select cognitive_robot`) so the interfaces are regenerated.

---

## Division of work

| Component                     | File(s)                                                                                                     | Owner |
|-------------------------------|-------------------------------------------------------------------------------------------------------------|-------|
| Abacus manipulation           | `abacus_manipulation_node.py`                                                                               | Mike  |
| Depth utilities               | `depth_utils.py`                                                                                            | Mike  |
| Abacus detection              | `detect_abacus_service.py`                                                                                  | Mike  |
| Station detection             | `detect_station_service.py`                                                                                 | Mike  |
| Clock OCR                     | `read_time_service.py`                                                                                      | Mike  |
| Mapping & station recognition | `plan_nav/trial_depth.py`                                                                                   | Ethan |
| Mission orchestration         | `plan_nav/station_demo.py`                                                                                  | Ethan |
| SLAM & Nav2                   | `config/`                                                                                                   | Bas   |
| Integration Gazebo & Robot    | `phase1_gazebo.launch.py`, `phase2_gazebo.launch.py` <br/> `phase1_real.launch.py`, `phase2_real.launch.py` | Bas |
| Gazebo demo map               | `gazebo_map_load/`                                                                                          | Ethan |

---

## Team notes

<!--
Teammates: add subsystem details, design decisions, known issues, or TODOs below.
Keep the tables above in sync when you add or rename a folder.
-->

_(add notes here)_

---

