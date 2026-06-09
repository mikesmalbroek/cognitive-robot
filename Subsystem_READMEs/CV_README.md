> **Note:** Commands use my specific directory paths — adjust them for your own setup.

---

# Computer Vision — Command Reference

---
3
## 0. First-Time Setup

Install the required Python packages (only needed once):

```bash
python3 -m pip install easyocr inference-sdk
```

> Use `python3 -m pip` (not plain `pip`) to ensure packages are installed into the same Python environment that ROS nodes use.

Then build the full workspace so ROS picks up all packages:

```bash
cd ~/mirte_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

> **If the build fails** with `failed to create symbolic link ... Is a directory` on `cognitive_robot_interfaces`, clear the stale build cache and retry:
> ```bash
> rm -rf ~/mirte_ws/build/cognitive_robot_interfaces
> rm -rf ~/mirte_ws/install/cognitive_robot_interfaces
> colcon build
> ```

- `easyocr` — time-reading service (OCR on digital clock)
- `inference-sdk` — abacus detection service (Roboflow)

---

## 1. Gazebo Simulation

Run all of these on your **laptop**, each in a separate terminal.

Run this in **every new terminal** before anything else:

```bash
cd ~/mirte_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

**Terminal 1 — Launch Gazebo**
```bash
killall gzserver gzclient 2>/dev/null
sleep 2
ros2 launch mirte_gazebo gazebo_mirte_master_empty.launch.xml
```

**Terminal 2 — Launch the Demo**
```bash
ros2 launch cognitive_robot demo_gazebo.launch.py
```

Or without Gazebo:
```bash
ros2 launch cognitive_robot demo.launch.py
```

**Terminal 3 — Keyboard Teleop** *(optional — to drive the robot manually)*
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/mirte_base_controller/cmd_vel_unstamped
```

**Terminal 4 — Call a CV Service**
```bash
# Abacus detection
ros2 service call /detect_abacus cognitive_robot_interfaces/srv/DetectAbacus '{}'

# Station / ArUco detection
ros2 service call /detect_station cognitive_robot_interfaces/srv/DetectStation '{}'

# Time reading
ros2 service call /read_time cognitive_robot_interfaces/srv/ReadTime
```

---

## 2. Real Robot

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

**`[ROBOT]` Terminal 1 — Verify the camera is visible**
```bash
ros2 topic list | grep camera
```
- If the list is empty: wrong `ROS_DOMAIN_ID`, or WiFi not connected correctly.
- If ROS nodes seem broken, restart the ROS service:
  ```bash
  sudo service mirte-ros restart   # password: mirte_mirte
  ```

**`[LAPTOP]` Terminal 2 — Launch the Demo**
```bash
ros2 launch cognitive_robot demo.launch.py
```

**`[LAPTOP]` Terminal 3 — Call a CV Service**
```bash
# Abacus detection
ros2 service call /detect_abacus cognitive_robot_interfaces/srv/DetectAbacus '{}'

# Station / ArUco detection
ros2 service call /detect_station cognitive_robot_interfaces/srv/DetectStation '{}'

# Time reading
ros2 service call /read_time cognitive_robot_interfaces/srv/ReadTime
```

---

### After a code change — rebuild and source

```bash
cd ~/mirte_ws
colcon build --packages-select cognitive_robot
source install/setup.bash
```