# cognitive_robot

ROS2 workspace packages for the MIRTE Master cognitive robot project.

## What this project does

When the task planner has driven the robot to a station, it calls the
`/read_time` ROS2 service.  The service:

1. Grabs a frame from the robot's front camera (`/camera/color/image_raw`).
2. Runs EasyOCR on the frame, looking for a `NN:NN` time string.
3. If nothing readable is found, rotates the robot slightly and tries again
   (alternating left/right with increasing angles).
4. Returns `found=True` and the four time digits (e.g. `[1, 4, 3, 2]` for
   `14:32`), or `found=False` if all attempts fail.

The OCR runs on the **laptop**, not the robot — the model is too heavy for
the robot's CPU.  The robot only provides the camera stream and accepts
rotation commands.

---

## Repository structure

```
cognitive_robot/                     ← git repo root
├── cognitive_robot/                 ← Python source files
│   ├── __init__.py
│   ├── read_time_service.py         ← main service node (OCR + rotation)
│   └── make_photo_for_testing_algorithm.py  ← debug tool (save photos manually)
│
├── cognitive_robot_interfaces/      ← custom ROS2 service definition
│   ├── srv/ReadTime.srv
│   ├── CMakeLists.txt
│   └── package.xml
│
├── launch/
├── resource/
├── test/
├── package.xml
├── setup.py
└── README.md
```

---

## Prerequisites

### ROS2 Humble
Install from the official instructions: https://docs.ros.org/en/humble/Installation.html

### Python pip dependencies
These are **not** managed by ROS — install them manually:

```bash
pip install easyocr opencv-python numpy
```

> EasyOCR will download its model files (~200 MB) on first run.

---

## Build

Always build from the workspace root (`~/ros2_ws`), not from inside the package:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

To build only these packages (faster during development):

```bash
colcon build --packages-select cognitive_robot_interfaces cognitive_robot
source install/setup.bash
```

> **Important:** build `cognitive_robot_interfaces` first (or together) because
> `cognitive_robot` depends on the generated service code.

---

## Network setup

The robot and laptop must be on the same network and use the same
`ROS_DOMAIN_ID`.  The default is `0`.

```bash
# Set this in every terminal before running any ROS2 commands
export ROS_DOMAIN_ID=0
```

Verify the camera topic is visible from the laptop:

```bash
ros2 topic list | grep camera
# Expected: /camera/color/image_raw
```

---

## Running the service

**Terminal 1 — start the service node on the laptop:**

```bash
source ~/ros2_ws/install/setup.bash
ros2 run cognitive_robot read_time_service
```

You will see:

```
[INFO] Loading EasyOCR model (this takes ~10 s)…
[INFO] EasyOCR model loaded.
[INFO] Debug images will be saved to: /home/<you>/ocr_debug
[INFO] Service /read_time is ready.
```

**Terminal 2 — simulate a task planner call:**

```bash
source ~/ros2_ws/install/setup.bash
ros2 service call /read_time cognitive_robot_interfaces/srv/ReadTime
```

Expected response on success:

```
response: cognitive_robot_interfaces.srv.ReadTime_Response(
    found=True,
    time_digits=[1, 4, 3, 2]
)
```

---

## Tuning parameters

Parameters can be passed at launch time with `--ros-args`:

```bash
ros2 run cognitive_robot read_time_service --ros-args \
    -p confidence_threshold:=0.8 \
    -p step_degrees:=15 \
    -p max_iterations:=8 \
    -p rotation_speed:=0.3 \
    -p debug_save_dir:=~/my_debug_folder
```

| Parameter             | Default      | Description |
|-----------------------|--------------|-------------|
| `confidence_threshold`| `0.7`        | Minimum OCR confidence to accept a detection. Raise if you get false positives; lower if valid times are rejected. |
| `step_degrees`        | `10`         | Degrees added per iteration when scanning. Smaller = finer search; larger = faster. |
| `max_iterations`      | `10`         | Maximum scan attempts before giving up. |
| `rotation_speed`      | `0.5` rad/s  | How fast the robot rotates. Slower = more accurate time-based rotation. |
| `debug_save_dir`      | `~/ocr_debug`| Where debug images are stored on the laptop. |

**Most important to tune: `confidence_threshold`.**  
If the service returns `found=False` even though the clock is visible, lower
it to `0.5`.  If it returns wrong times, raise it to `0.85`.

---

## Debug images

After each service call, `~/ocr_debug/` (or your configured directory) contains
one pair of files per scan iteration:

```
attempt_20240514_103045_123456_iter0.jpg   ← camera frame
attempt_20240514_103045_123456_iter0.txt   ← OCR results
```

Example `.txt` content:

```
Iteration : 0
Timestamp : 20240514_103045_123456
Threshold : 0.7

--- OCR detections ---
  [0] text="14:32"  confidence=0.981  format_ok=True  conf_ok=True  → ACCEPTED
  [1] text="6006444"  confidence=0.091  format_ok=False  conf_ok=False  → rejected
```

Use these files to understand why the service succeeded or failed.

---

## Debug tool: manually save photos

To collect sample images for offline OCR testing:

```bash
ros2 run cognitive_robot make_photo_for_testing_algorithm
# Press Enter to save a frame, type 'q' to quit.
# Photos saved to ~/photos/
```

This tool listens to the **gripper camera** (`/gripper_camera/image_raw`).
It is not part of the production pipeline — only for testing.
