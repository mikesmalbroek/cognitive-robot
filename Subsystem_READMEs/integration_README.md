> **Note:** Commands use my specific directory paths — adjust them for your own setup.

---

# Integration — Command Reference

Each phase is one launch command that starts everything at once.

---

## Pipeline overview

### Phase 1: Manual map generation
1. Launch Phase 1 (Gazebo or real robot)
2. Drive around the environment to build the map
3. Drive to Station A, press **B** to register it
4. Drive to Station B, press **B** to register it
5. Press **V** to save the map and quit

### Phase 2: Autonomous mission
1. Place robot somewhere on the saved map
2. Launch Phase 2 (Gazebo or real robot)
3. Set the 2D pose estimate in RViz
4. Robot drives to Station A → reads the clock → drives to Station B automatically

### Phase 3: Manipulation (manual)
- Currently done with manual commands
- Will be attempted autonomously if time permits

---

## 0. First-Time Setup

Build the workspace so ROS picks up all packages (only needed after code changes):

```bash
cd ~/mirte_ws
colcon build --packages-select cognitive_robot
```

Run this in **every new terminal** before anything else:

```bash
cd ~/mirte_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

For **real robot** terminals, also add:

```bash
export ROS_DOMAIN_ID=4
```

---

## 1. Phase 1 — Gazebo (Manual Map Generation)

Starts: Gazebo + SLAM + RViz + CV services + keyboard teleop window.

```bash
# [LAPTOP]
ros2 launch cognitive_robot phase1_gazebo.launch.py
```

**Controls** (click the OpenCV camera window first):

| Key   | Action                           |
|-------|----------------------------------|
| W / S | Forward / back                   |
| A / D | Strafe left / right              |
| Q / E | Rotate left / right              |
| X     | Stop                             |
| B     | Detect + register nearest station |
| V     | Save map and quit                |
| ESC   | Quit without saving              |

Drive to Station A, press **B**. Drive to Station B, press **B**. When the map looks good, press **V**.

Saved files go to: `~/mirte_ws/src/cognitive-robot/maps/`

---

## 2. Phase 1 — Real Robot (Manual Map Generation)

**Before you start:**
- Connect laptop WiFi to: `Mirte-XXXXXX` (password: `mirte_mirte`)
- SSH into the robot:
  ```bash
  ssh mirte@172.20.10.4
  # password: mirte_mirte
  ```
- Set domain ID (do this in **every** terminal for real robot):
  ```bash
  export ROS_DOMAIN_ID=4
  ```
- Verify the robot is publishing topics:
  ```bash
  ros2 topic list   # expect /scan, /mirte_base_controller/cmd_vel, etc.
  ```

```bash
# [LAPTOP]
export ROS_DOMAIN_ID=4
ros2 launch cognitive_robot phase1_real.launch.py
```

Same controls as the Gazebo version above.

---

## 3. Phase 2 — Gazebo (Autonomous Mission)

> **Requires:** Phase 1 Gazebo must be complete. `station_a_location.yaml`, `station_b_location.yaml`, and `auto_map.yaml` must exist in `maps/`.

Starts: Gazebo + Nav2 + RViz + CV services + autonomous mission node.

```bash
# [LAPTOP]
ros2 launch cognitive_robot phase2_gazebo.launch.py
```

To use a different map:
```bash
ros2 launch cognitive_robot phase2_gazebo.launch.py map:=/full/path/to/map.yaml
```

**After launch — set the robot's starting position in RViz:**
1. Click **"2D Pose Estimate"** at the top of RViz
2. Click on the map where the robot is
3. Drag in the direction the robot is facing

> The laser scan should align with the walls. Once it does, the mission starts automatically.

---

## 4. Phase 2 — Real Robot (Autonomous Mission)

> **Requires:** Phase 1 Real must be complete. Station YAML files and map must be saved.

**Before you start:**
- Connect laptop WiFi to: `Mirte-XXXXXX` (password: `mirte_mirte`)
- SSH into the robot:
  ```bash
  ssh mirte@172.20.10.4
  # password: mirte_mirte
  ```
- Set domain ID (do this in **every** terminal for real robot):
  ```bash
  export ROS_DOMAIN_ID=4
  ```
- Place the robot somewhere on the saved map

```bash
# [LAPTOP]
export ROS_DOMAIN_ID=4
ros2 launch cognitive_robot phase2_real.launch.py
```

**After launch — set the robot's starting position in RViz:**
1. Click **"2D Pose Estimate"** at the top of RViz
2. Click on the map where the robot is
3. Drag in the direction the robot is facing

> The mission then runs automatically: drive to Station A → read the clock → drive to Station B.

> **Note:** Nav2 runs on the laptop here (not the robot), communicating over `ROS_DOMAIN_ID=4`.
> If navigation does not work, fall back to starting Nav2 on the robot manually:
> ```bash
> # on the robot (via SSH)
> ros2 launch mirte_navigation minimal_navigation_launch.py
> ```
> Then run only the CV services + station_demo on the laptop:
> ```bash
> ros2 launch cognitive_robot demo.launch.py
> ros2 run cognitive_robot station_demo
> ```