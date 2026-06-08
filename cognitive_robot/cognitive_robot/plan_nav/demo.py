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


STATION_A_FILE = "station_a_location.yaml"
STATION_B_FILE = "station_b_location.yaml"

WAIT_BETWEEN_STATIONS_SEC = 5.0


def yaw_to_quaternion(yaw):
    """
    Convert planar yaw angle to quaternion.
    """
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)

    return {
        "x": 0.0,
        "y": 0.0,
        "z": qz,
        "w": qw,
    }


def load_station_destination(yaml_file):
    """
    Load destination_pose from station YAML file.

    Expected YAML structure:

    destination_pose:
      frame_id: "map"
      x: ...
      y: ...
      z: ...
      yaw_rad: ...
    """

    if not os.path.exists(yaml_file):
        raise FileNotFoundError(f"Could not find station file: {yaml_file}")

    with open(yaml_file, "r") as f:
        data = yaml.safe_load(f)

    station_name = data.get("station_name", yaml_file)

    if "destination_pose" not in data:
        raise KeyError(
            f"{yaml_file} does not contain destination_pose. "
            f"Press b again with the updated station recorder code."
        )

    dest = data["destination_pose"]

    frame_id = dest.get("frame_id", "map")
    x = float(dest["x"])
    y = float(dest["y"])
    z = float(dest.get("z", 0.0))
    yaw = float(dest["yaw_rad"])

    return {
        "station_name": station_name,
        "frame_id": frame_id,
        "x": x,
        "y": y,
        "z": z,
        "yaw": yaw,
    }


class StationNavigator(Node):
    def __init__(self):
        super().__init__("station_navigator")

        self.client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose"
        )

        self.goal_done = False
        self.goal_succeeded = False

    def wait_for_nav2(self):
        self.get_logger().info("Waiting for Nav2 /navigate_to_pose action server...")

        self.client.wait_for_server()

        self.get_logger().info("Nav2 action server is available.")

    def send_goal_and_wait(self, station_goal):
        self.goal_done = False
        self.goal_succeeded = False

        station_name = station_goal["station_name"]
        frame_id = station_goal["frame_id"]
        x = station_goal["x"]
        y = station_goal["y"]
        z = station_goal["z"]
        yaw = station_goal["yaw"]

        quat = yaw_to_quaternion(yaw)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = z

        goal_msg.pose.pose.orientation.x = quat["x"]
        goal_msg.pose.pose.orientation.y = quat["y"]
        goal_msg.pose.pose.orientation.z = quat["z"]
        goal_msg.pose.pose.orientation.w = quat["w"]

        self.get_logger().info("=" * 70)
        self.get_logger().info(f"Sending goal to {station_name}")
        self.get_logger().info(f"Frame : {frame_id}")
        self.get_logger().info(f"x     : {x:+.3f} m")
        self.get_logger().info(f"y     : {y:+.3f} m")
        self.get_logger().info(f"yaw   : {math.degrees(yaw):+.1f} deg")
        self.get_logger().info("=" * 70)

        send_goal_future = self.client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()

        if goal_handle is None:
            self.get_logger().error("Goal handle is None. Failed to send goal.")
            return False

        if not goal_handle.accepted:
            self.get_logger().error(f"Goal to {station_name} was rejected by Nav2.")
            return False

        self.get_logger().info(f"Goal to {station_name} accepted.")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()

        if result is None:
            self.get_logger().error(f"No result received for {station_name}.")
            return False

        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Successfully reached {station_name}.")
            return True

        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error(f"Navigation to {station_name} was aborted.")
            return False

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().error(f"Navigation to {station_name} was canceled.")
            return False

        else:
            self.get_logger().warn(
                f"Navigation to {station_name} finished with status: {status}"
            )
            return False

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        distance_remaining = feedback.distance_remaining
        navigation_time = feedback.navigation_time.sec

        self.get_logger().info(
            f"Distance remaining: {distance_remaining:.3f} m | "
            f"Navigation time: {navigation_time} s"
        )


def main(args=None):
    rclpy.init(args=args)

    navigator = StationNavigator()

    try:
        station_a_goal = load_station_destination(STATION_A_FILE)
        station_b_goal = load_station_destination(STATION_B_FILE)

        navigator.get_logger().info("Loaded station destination files:")
        navigator.get_logger().info(f"  Station A file: {STATION_A_FILE}")
        navigator.get_logger().info(f"  Station B file: {STATION_B_FILE}")

        navigator.wait_for_nav2()

        success_a = navigator.send_goal_and_wait(station_a_goal)

        if not success_a:
            navigator.get_logger().error(
                "Failed to reach Station A. Not continuing to Station B."
            )
            return

        navigator.get_logger().info(
            f"Waiting {WAIT_BETWEEN_STATIONS_SEC:.1f} seconds before going to Station B..."
        )
        time.sleep(WAIT_BETWEEN_STATIONS_SEC)

        success_b = navigator.send_goal_and_wait(station_b_goal)

        if not success_b:
            navigator.get_logger().error("Failed to reach Station B.")
            return

        navigator.get_logger().info("Mission complete: Station A then Station B reached.")

    except Exception as e:
        navigator.get_logger().error(f"Station navigation failed: {e}")

    finally:
        navigator.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

