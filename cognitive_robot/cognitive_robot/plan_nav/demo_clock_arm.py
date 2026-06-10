#!/usr/bin/env python3

import math
import os
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand

from cognitive_robot_interfaces.srv import ReadTime


_HERE = os.path.dirname(os.path.abspath(__file__))

STATION_A_FILE = os.path.join(_HERE, "station_a_location.yaml")
STATION_B_FILE = os.path.join(_HERE, "station_b_location.yaml")

READ_TIME_SERVICE = "/read_time"

ARM_TOPIC = "/mirte_master_arm_controller/joint_trajectory"
GRIPPER_ACTION = "/mirte_master_gripper_controller/gripper_cmd"
CMD_VEL_TOPIC = "/mirte_base_controller/cmd_vel_unstamped"

USE_FAKE_TIME = False
FAKE_TIME_DIGITS = [1, 2, 0, 1]

ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_joint",
]

ARM_VERTICAL = [
    0.0,
    0.0,
    0.0,
    0.0,
]

ARM_ELBOW_DOWN = [
    0.0,
    0.0,
    -math.pi / 2.0,
    0.0,
]

ARM_MOVE_TIME = 1.5

GRIPPER_OPEN_POS = 0.00
GRIPPER_CLOSED_POS = 0.15
GRIPPER_MAX_EFFORT = 10.0

WAIT_FOR_RING_SEC = 3.0
WAIT_AFTER_RELEASE_SEC = 0.2
HORIZONTAL_SPEED = 0.10
HORIZONTAL_MOVE_TIME = 0.5


def yaw_to_quaternion(yaw):
    return (
        0.0,
        0.0,
        math.sin(yaw / 2.0),
        math.cos(yaw / 2.0),
    )


def load_station_goal(filename):
    with open(filename, "r") as f:
        data = yaml.safe_load(f)

    dest = data["destination_pose"]

    return {
        "name": data.get("station_name", filename),
        "frame_id": dest.get("frame_id", "map"),
        "x": float(dest["x"]),
        "y": float(dest["y"]),
        "z": float(dest.get("z", 0.0)),
        "yaw": float(dest["yaw_rad"]),
    }


def digits_to_time(digits):
    if digits is None or len(digits) != 4:
        return "unknown"

    return f"{digits[0]}{digits[1]}:{digits[2]}{digits[3]}"


class FullStationAbacusMission(Node):
    def __init__(self):
        super().__init__("full_station_abacus_mission")

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose"
        )

        self.read_time_client = self.create_client(
            ReadTime,
            READ_TIME_SERVICE
        )

        self.arm_pub = self.create_publisher(
            JointTrajectory,
            ARM_TOPIC,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            CMD_VEL_TOPIC,
            10
        )

        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            GRIPPER_ACTION
        )

        time.sleep(1.0)


    def go_to(self, goal):
        self.get_logger().info(f"Going to {goal['name']}")

        qx, qy, qz, qw = yaw_to_quaternion(goal["yaw"])

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose.header.frame_id = goal["frame_id"]
        nav_goal.pose.header.stamp = self.get_clock().now().to_msg()

        nav_goal.pose.pose.position.x = goal["x"]
        nav_goal.pose.pose.position.y = goal["y"]
        nav_goal.pose.pose.position.z = goal["z"]

        nav_goal.pose.pose.orientation.x = qx
        nav_goal.pose.pose.orientation.y = qy
        nav_goal.pose.pose.orientation.z = qz
        nav_goal.pose.pose.orientation.w = qw

        self.get_logger().info("Waiting for Nav2 /navigate_to_pose server...")
        self.nav_client.wait_for_server()

        send_future = self.nav_client.send_goal_async(nav_goal)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()

        if goal_handle is None:
            self.get_logger().error(f"No goal handle received for {goal['name']}")
            return False

        if not goal_handle.accepted:
            self.get_logger().error(f"Goal rejected: {goal['name']}")
            return False

        self.get_logger().info(f"Goal accepted: {goal['name']}")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()

        if result is None:
            self.get_logger().error(f"No Nav2 result for {goal['name']}")
            return False

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Reached {goal['name']}")
            return True

        self.get_logger().error(f"Failed to reach {goal['name']}, status={result.status}")
        return False

    # ------------------------------------------------------------
    # Read time
    # ------------------------------------------------------------

    def read_time(self):
        if USE_FAKE_TIME:
            self.get_logger().warn(
                f"Using fake time digits: {digits_to_time(FAKE_TIME_DIGITS)}"
            )
            return FAKE_TIME_DIGITS

        self.get_logger().info("Reading time at Station A...")

        self.read_time_client.wait_for_service()

        request = ReadTime.Request()
        future = self.read_time_client.call_async(request)

        rclpy.spin_until_future_complete(self, future)

        response = future.result()

        if response is None:
            self.get_logger().error("No response from /read_time")
            return None

        if not response.found:
            self.get_logger().warn("Time not detected.")
            return None

        digits = list(response.time_digits)

        self.get_logger().info(f"Detected time: {digits_to_time(digits)}")
        self.get_logger().info(f"Detected digits: {digits}")

        return digits

    # ------------------------------------------------------------
    # Arm control
    # ------------------------------------------------------------

    def send_arm_pose(self, positions):
        msg = JointTrajectory()
        msg.joint_names = ARM_JOINTS

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(ARM_MOVE_TIME)
        point.time_from_start.nanosec = int((ARM_MOVE_TIME - int(ARM_MOVE_TIME)) * 1e9)

        msg.points.append(point)
        self.arm_pub.publish(msg)

        self.get_logger().info(
            f"Arm command: "
            f"pan={positions[0]:+.3f}, "
            f"shoulder_lift={positions[1]:+.3f}, "
            f"elbow={positions[2]:+.3f}, "
            f"wrist={positions[3]:+.3f}"
        )

        time.sleep(ARM_MOVE_TIME + 0.2)

    def set_vertical(self):
        self.get_logger().info("Setting arm vertical")
        self.send_arm_pose(ARM_VERTICAL)

    def move_elbow_down(self):
        self.get_logger().info("Moving elbow down 90 degrees")
        self.send_arm_pose(ARM_ELBOW_DOWN)

    # ------------------------------------------------------------
    # Gripper control
    # ------------------------------------------------------------

    def send_gripper_position(self, position):
        self.gripper_client.wait_for_server()

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = GRIPPER_MAX_EFFORT

        send_future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()

        if goal_handle is None:
            self.get_logger().error("No gripper goal handle received.")
            return False

        if not goal_handle.accepted:
            self.get_logger().error("Gripper goal rejected.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        return True

    def open_gripper(self):
        self.get_logger().info("Opening gripper")
        self.send_gripper_position(GRIPPER_OPEN_POS)

    def close_gripper(self):
        self.get_logger().info("Closing gripper")
        self.send_gripper_position(GRIPPER_CLOSED_POS)

    # ------------------------------------------------------------
    # Base movement
    # ------------------------------------------------------------

    def stop_robot(self):
        msg = Twist()
        self.cmd_pub.publish(msg)
        time.sleep(0.2)

    def move_right(self):
        self.get_logger().info("Moving robot horizontally right")

        msg = Twist()

        # ROS robot frame:
        # +y = left
        # -y = right
        msg.linear.y = -HORIZONTAL_SPEED

        start_time = time.time()

        while time.time() - start_time < HORIZONTAL_MOVE_TIME:
            self.cmd_pub.publish(msg)
            time.sleep(0.05)

        self.stop_robot()

        self.get_logger().info("Finished horizontal right movement")

    # ------------------------------------------------------------
    # Ring placement
    # ------------------------------------------------------------

    def place_one_ring(self):
        self.get_logger().info("Starting one ring placement")

        # 1. Start vertical
        self.set_vertical()

        # 2. Open gripper for 3 seconds to receive the ring
        self.open_gripper()
        self.get_logger().info("Waiting 3 seconds to receive ring")
        time.sleep(WAIT_FOR_RING_SEC)

        # 3. Close gripper to hold the ring
        self.close_gripper()

        # 4. Move elbow down 90 degrees
        self.move_elbow_down()

        # 5. Open gripper to release ring
        self.open_gripper()
        time.sleep(WAIT_AFTER_RELEASE_SEC)

        # 6. Lift elbow back up to vertical
        self.set_vertical()

        self.get_logger().info("Finished one ring placement")

    def place_digit(self, digit, column_index):
        self.get_logger().info(
            f"Column {column_index + 1}: placing {digit} rings"
        )

        for ring_index in range(digit):
            self.get_logger().info(
                f"Column {column_index + 1}, ring {ring_index + 1}/{digit}"
            )
            self.place_one_ring()

    def place_time_on_abacus(self, digits):
        if digits is None:
            self.get_logger().error("No detected digits. Skipping abacus placement.")
            return

        if len(digits) != 4:
            self.get_logger().error(f"Expected 4 digits, got: {digits}")
            return

        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Placing time on abacus: {digits_to_time(digits)}")
        self.get_logger().info("=" * 60)

        self.set_vertical()

        for column_index, digit in enumerate(digits):
            self.place_digit(digit, column_index)

            if column_index < 3:
                self.move_right()

        self.set_vertical()

        self.get_logger().info("Finished placing rings on abacus")

    # ------------------------------------------------------------
    # Full mission
    # ------------------------------------------------------------

    def run_mission(self):
        station_a = load_station_goal(STATION_A_FILE)
        station_b = load_station_goal(STATION_B_FILE)

        self.get_logger().info("Loaded station files")
        self.get_logger().info(f"Station A: {STATION_A_FILE}")
        self.get_logger().info(f"Station B: {STATION_B_FILE}")

        # 1. Travel to Station A
        if not self.go_to(station_a):
            return

        # 2. Read time at Station A
        digits = self.read_time()

        if digits is None:
            self.get_logger().error("No time detected. Stopping mission.")
            return

        # 3. Travel to Station B
        if not self.go_to(station_b):
            return

        # 4. Place rings according to detected time
        self.place_time_on_abacus(digits)

        self.get_logger().info("=" * 60)
        self.get_logger().info("MISSION COMPLETE")
        self.get_logger().info(f"Final time represented: {digits_to_time(digits)}")
        self.get_logger().info("=" * 60)


def main(args=None):
    rclpy.init(args=args)

    node = FullStationAbacusMission()

    try:
        node.run_mission()

    except KeyboardInterrupt:
        pass

    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()