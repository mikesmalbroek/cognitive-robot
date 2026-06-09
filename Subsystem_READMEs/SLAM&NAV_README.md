> **Note:** The following commands use my specific directory paths — adjust them to match your own setup.
>
> Also, replace the following file with the version in this same folder:
> `/mirte_ws/src/mirte-ros-packages/mirte_description/mirte_master_description/urdf/mirte_master.xacro`

---

# MIRTE MASTER — SLAM Command Reference

---

## Section 1: Gazebo Simulation

Run all of these on your **laptop**, each in a separate terminal. Order matters: launch Gazebo before SLAM, SLAM before RViz.

Run this in **every new terminal** before anything else:

```bash
cd ~/mirte_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

**Terminal 1 — Launch Gazebo**
```bash
ros2 launch mirte_gazebo gazebo_mirte_master_empty.launch.xml
```

**Terminal 2 — Launch SLAM Toolbox**
```bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=/home/bas/mirte_ws/src/cognitive-robot/config/mapper_params_online_async.yaml \
  use_sim_time:=true
```

**Terminal 3 — Launch RViz**
```bash
rviz2 -d /home/bas/mirte_ws/src/cognitive-robot/config/mirte_slam.rviz
```

**Terminal 4 — Teleoperation**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Terminal 5 — Save map** *(run when map looks good, before stopping SLAM)*
```bash
ros2 run nav2_map_server map_saver_cli -f ~/mirte_ws/src/cognitive-robot/maps/my_map
```

---

## Section 2: Real Robot

- `[LAPTOP]` — run on your laptop
- `[ROBOT]` — run on the robot (via SSH or web editor)

**Before you start:**
- Connect laptop WiFi to: `Mirte-XXXXXX` (password: `mirte_mirte`)
- Open browser: `http://192.168.42.1:8000` (user: `mirte` / pass: `mirte_mirte`)

SSH into the robot:
```bash
ssh mirte@172.20.10.4
```

Run this in **every new `[LAPTOP]` terminal** before anything else:
```bash
cd ~/mirte_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=4
```

Run this in **every new `[ROBOT]` terminal** before anything else:
```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=4
```

---

**`[ROBOT]` Terminal 1 — Verify robot is publishing topics**
```bash
ros2 topic list
```
- You should see `/scan`, `/mirte_base_controller/cmd_vel`, `/mirte_base_controller/odom`, etc.
- If the list is empty: wrong `ROS_DOMAIN_ID`, or WiFi not connected correctly.
- If ROS nodes seem broken, restart the ROS service:
  ```bash
  sudo service mirte-ros restart   # password: mirte_mirte
  ```

**`[LAPTOP]` Terminal 2 — Launch SLAM Toolbox**
```bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=/home/bas/mirte_ws/src/cognitive-robot/config/mapper_params_online_async.yaml \
  use_sim_time:=false
```

**`[LAPTOP]` Terminal 3 — Launch RViz**
```bash
rviz2 -d /home/bas/mirte_ws/src/cognitive-robot/config/mirte_slam.rviz
```
- In RViz: Global Options > Fixed Frame > set to `map`
- Add displays: Map (`/map`), LaserScan (`/scan`), RobotModel (`/robot_description`)

**`[LAPTOP]` Terminal 4 — Teleoperation**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/mirte_base_controller/cmd_vel
```
- Drive slowly for a clean map. Avoid sharp/fast turns.

**`[LAPTOP]` Terminal 5 — Save map** *(run when map looks good, before stopping SLAM)*
```bash
ros2 run nav2_map_server map_saver_cli -f ~/mirte_ws/src/cognitive-robot/maps/my_map
```
- Saves: `my_map.pgm` (image) and `my_map.yaml` (metadata)
- Re-running this command overwrites the previous save.

---

## Section 3: Useful Debug Commands (Real Robot)

| Task | Command |
| --- | --- |
| Check all active topics | `ros2 topic list` |
| Verify laser scan | `ros2 topic echo /scan --once` |
| Verify odometry | `ros2 topic echo /mirte_base_controller/odom --once` |
| Verify cmd_vel (while driving) | `ros2 topic echo /mirte_base_controller/cmd_vel` |
| Check TF tree (saves frames.pdf) | `ros2 run tf2_tools view_frames` |
| Check specific TF transform | `ros2 run tf2_ros tf2_echo odom base_link` |

---

## Section 4: Gazebo — Nav2 Navigation

Replaces SLAM entirely. Launches Gazebo, Nav2, and RViz in one command. Make sure SLAM is **not** running.

Run this in **every new terminal** before anything else:
```bash
cd ~/mirte_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

**Terminal 1 — Launch Gazebo + Nav2 + RViz**
```bash
ros2 launch spatial_ai_navigation spatial_ai_navigation.launch.py \
  map:=src/cognitive-robot/maps/my_map.yaml
```

**After launch — Set initial pose in RViz**
1. Click **"2D Pose Estimate"** at the top of RViz
2. Click on the map where the robot is located
3. Drag in the direction the robot is facing

> The laser scan should align with the walls on the map.

**Send a navigation goal from RViz**
1. Click **"Nav2 Goal"** at the top of RViz
2. Click the target position on the map
3. Drag to set the final heading direction

**Send a navigation goal from the command line**
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: 1.0, y: 2.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  },
  behavior_tree: ''
}" --feedback
```
- Change `x` and `y` to your desired goal coordinates.

---

## Section 5: Real Robot — Nav2 Navigation

Stop SLAM (Terminal 2) and Teleop (Terminal 4) before starting. Keep RViz open.

**Before you start — copy your map to the robot** *(run once from a `[LAPTOP]` terminal)*:
```bash
scp ~/mirte_ws/src/cognitive-robot/maps/my_map.yaml mirte@10.121.167.158:/home/mirte/mirte_ws/src/mirte_navigation/maps/
scp ~/mirte_ws/src/cognitive-robot/maps/my_map.pgm mirte@10.121.167.158:/home/mirte/mirte_ws/src/mirte_navigation/maps/
```

**Update the map path on the robot** *(run once from a `[ROBOT]` terminal)*:
```bash
nano ~/mirte_ws/src/mirte_navigation/params/minimal_nav2_params.yaml
```
Find and update this line:
```yaml
yaml_filename: "/home/mirte/mirte_ws/src/mirte_navigation/maps/my_map.yaml"
```

---

**`[ROBOT]` Terminal 1 — Launch Nav2**
```bash
ros2 launch mirte_navigation minimal_navigation_launch.py
```

**`[LAPTOP]` Terminal 3 — RViz is already open**

Add displays if not already present: Map (`/map`), LaserScan (`/scan`), Global Costmap, Local Costmap.

Set initial pose:
1. Click **"2D Pose Estimate"** at the top of RViz
2. Click on the map where the robot is located
3. Drag in the direction the robot is facing

**`[LAPTOP]` — Send a navigation goal from RViz**
1. Click **"Nav2 Goal"**
2. Click the target on the map
3. Drag to set heading

**`[LAPTOP]` — Send a navigation goal from the command line**
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: 1.0, y: 2.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  },
  behavior_tree: ''
}" --feedback
```
- Change `x` and `y` to your desired goal coordinates.