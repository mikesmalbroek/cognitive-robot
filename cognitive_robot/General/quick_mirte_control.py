#!/usr/bin/env python3

# ============================================================
# IMPORTS
# ============================================================
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import time
import math


# ============================================================
# SIMPLE MIRTE CONTROL NODE
# ============================================================
class QuickMirteControl(Node):

    def __init__(self):
        super().__init__("quick_mirte_control")

        # ------------------------------------------------------------
        # Publisher for robot base movement
        # ------------------------------------------------------------
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            "/mirte_base_controller/cmd_vel_unstamped",
            10
        )

        # ------------------------------------------------------------
        # Publisher for robot arm movement
        # ------------------------------------------------------------
        self.arm_pub = self.create_publisher(
            JointTrajectory,
            "/mirte_master_arm_controller/joint_trajectory",
            10
        )

        self.get_logger().info("Quick MIRTE control node started")

        # Small delay so publishers connect properly
        time.sleep(1.0)

        # Run the full movement sequence
        self.run_sequence()

    # ============================================================
    # FUNCTION: MOVE BASE
    # ============================================================
    def move_base(self, linear_x, angular_z, duration):
        """
        Sends velocity commands to the robot base.

        linear_x:
            Positive = forward
            Negative = backward

        angular_z:
            Positive = rotate left
            Negative = rotate right

        duration:
            How long the command is sent, in seconds
        """

        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z

        start_time = time.time()

        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(msg)
            time.sleep(0.05)

        self.stop_robot()

    # ============================================================
    # FUNCTION: STOP ROBOT
    # ============================================================
    def stop_robot(self):
        """
        Stops all base motion.
        """

        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0

        self.cmd_vel_pub.publish(msg)
        time.sleep(0.5)

    # ============================================================
    # FUNCTION: MOVE ARM
    # ============================================================
    def move_arm(self, shoulder_pan, shoulder_lift, elbow, wrist, duration=3):
        """
        Sends a joint trajectory command to the MIRTE arm.

        Joint order:
        1. shoulder_pan_joint
        2. shoulder_lift_joint
        3. elbow_joint
        4. wrist_joint
        """

        trajectory = JointTrajectory()

        trajectory.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_joint"
        ]

        point = JointTrajectoryPoint()
        point.positions = [
            shoulder_pan,
            shoulder_lift,
            elbow,
            wrist
        ]

        point.time_from_start.sec = duration
        point.time_from_start.nanosec = 0

        trajectory.points.append(point)

        self.arm_pub.publish(trajectory)

        time.sleep(duration + 0.5)

    # ============================================================
    # FUNCTION: FULL CONTROL SEQUENCE
    # ============================================================
    def run_sequence(self):

        # ------------------------------------------------------------
        # PART 1: Move robot forward
        # ------------------------------------------------------------
        self.get_logger().info("PART 1: Moving forward")

        self.move_base(
            linear_x=3.15,
            angular_z=0.0,
            duration=10.0
        )

        # ------------------------------------------------------------
        # PART 2: Lift the arm
        # ------------------------------------------------------------
        self.get_logger().info("PART 2: Lifting arm")

        self.move_arm(
            shoulder_pan=0.0,
            shoulder_lift=0.0,
            elbow=-1.56,
            wrist=1.56,
            duration=3
        )

        # ------------------------------------------------------------
        # PART 3: Move robot backward
        # ------------------------------------------------------------
        self.get_logger().info("PART 3: Moving backward")

        self.move_base(
            linear_x=-0.15,
            angular_z=0.0,
            duration=3.0
        )

        # ------------------------------------------------------------
        # PART 4: Rotate robot 90 degrees left
        # ------------------------------------------------------------
        self.get_logger().info("PART 4: Rotating 90 degrees left")

        angular_speed = 0.5  # rad/s
        target_angle = math.pi / 2  # 90 degrees in radians
        rotation_time = target_angle / angular_speed

        self.move_base(
            linear_x=0.0,
            angular_z=angular_speed,
            duration=rotation_time
        )

        # ------------------------------------------------------------
        # PART 5: Stop robot
        # ------------------------------------------------------------
        self.get_logger().info("PART 5: Stopping robot")

        self.stop_robot()

        self.get_logger().info("Sequence complete")


# ============================================================
# MAIN FUNCTION
# ============================================================
def main(args=None):
    rclpy.init(args=args)

    node = QuickMirteControl()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()