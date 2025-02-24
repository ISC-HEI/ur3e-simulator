#!/usr/bin/env python3

# ISCoin basic joint trajectory controller
# Based on: ur_robot_driver/scripts/example_move.py
# https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble

import time

import rclpy
from rclpy.action import ActionClient

from builtin_interfaces.msg import Duration
from action_msgs.msg import GoalStatus
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from ament_index_python import get_package_share_directory
import json
import os

class JointTrajectoryControllerClient(rclpy.node.Node):
    """Small client for the Joint Trajectory Controller (JTC)"""

    def __init__(self):
        super().__init__("jtc_client")
        # Declare default values for parameters
        self.declare_parameter("controller_name", "joint_trajectory_controller")
        self.declare_parameter(
            "joints",
            [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
        )
        self.declare_parameter("traj",
                               os.path.join(
                                   get_package_share_directory("iscoin_driver"),
                                   "config",
                                   "traj0.json"
                                ))

        # Update parameters based on parameter file or command line args
        controller_name = self.get_parameter("controller_name").value + "/follow_joint_trajectory"
        self.joints = self.get_parameter("joints").value
        self.traj_config_path = self.get_parameter("traj").value

        if self.joints is None or len(self.joints) == 0:
            raise Exception('"joints" parameter is required')

        self._action_client = ActionClient(self, FollowJointTrajectory, controller_name)
        self.get_logger().info(f"Waiting for action server on {controller_name}")
        self._action_client.wait_for_server()

        self.parse_trajectories()
        self.i = 0
        self._send_goal_future = None
        self._get_result_future = None
        self.execute_next_trajectory()

    def parse_trajectories(self):
        self.goals = {}

        with open(self.traj_config_path, "r") as fid:
            trajectories = json.load(fid)

        for traj_name in trajectories:
            goal = JointTrajectory()
            goal.joint_names = self.joints
            for pt in trajectories[traj_name]:
                point = JointTrajectoryPoint()
                point.positions = pt["positions"]
                point.velocities = pt["velocities"]
                point.time_from_start = Duration(
                    sec=pt["time_from_start"][0],
                    nanosec=pt["time_from_start"][1]
                )
                goal.points.append(point)

            self.goals[traj_name] = goal

    def execute_next_trajectory(self):
        if self.i >= len(self.goals):
            self.get_logger().info("Done with all trajectories")
            raise SystemExit
        traj_name = list(self.goals)[self.i]
        self.i = self.i + 1
        if traj_name:
            self.execute_trajectory(traj_name)

    def execute_trajectory(self, traj_name):
        self.get_logger().info(f"Executing trajectory {traj_name}")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self.goals[traj_name]

        goal.goal_time_tolerance = Duration(sec=0, nanosec=500000000)
        goal.goal_tolerance = [
            JointTolerance(position=0.01, velocity=0.01, name=self.joints[i]) for i in range(len(self.joints))
        ]

        self._send_goal_future = self._action_client.send_goal_async(goal)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected :(")
            raise RuntimeError("Goal rejected :(")

        self.get_logger().debug("Goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f"Done with result: {self.status_to_str(status)}")
        if status == GoalStatus.STATUS_SUCCEEDED:
            time.sleep(2)
            self.execute_next_trajectory()
        else:
            if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
                self.get_logger().error(
                    f"Done with result: {self.error_code_to_str(result.error_code)}"
                )
            raise RuntimeError("Executing trajectory failed. " + result.error_string)

    @staticmethod
    def error_code_to_str(error_code):
        if error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            return "SUCCESSFUL"
        if error_code == FollowJointTrajectory.Result.INVALID_GOAL:
            return "INVALID_GOAL"
        if error_code == FollowJointTrajectory.Result.INVALID_JOINTS:
            return "INVALID_JOINTS"
        if error_code == FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP:
            return "OLD_HEADER_TIMESTAMP"
        if error_code == FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED:
            return "PATH_TOLERANCE_VIOLATED"
        if error_code == FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED:
            return "GOAL_TOLERANCE_VIOLATED"

    @staticmethod
    def status_to_str(error_code):
        if error_code == GoalStatus.STATUS_UNKNOWN:
            return "UNKNOWN"
        if error_code == GoalStatus.STATUS_ACCEPTED:
            return "ACCEPTED"
        if error_code == GoalStatus.STATUS_EXECUTING:
            return "EXECUTING"
        if error_code == GoalStatus.STATUS_CANCELING:
            return "CANCELING"
        if error_code == GoalStatus.STATUS_SUCCEEDED:
            return "SUCCEEDED"
        if error_code == GoalStatus.STATUS_CANCELED:
            return "CANCELED"
        if error_code == GoalStatus.STATUS_ABORTED:
            return "ABORTED"


def main(args=None):
    rclpy.init(args=args)

    jtc_client = JointTrajectoryControllerClient()
    try:
        rclpy.spin(jtc_client)
    except RuntimeError as err:
        jtc_client.get_logger().error(str(err))
    except SystemExit:
        rclpy.logging.get_logger("jtc_client").info("Done")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
