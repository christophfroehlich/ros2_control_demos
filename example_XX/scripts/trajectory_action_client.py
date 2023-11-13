#!/usr/bin/env python3
# Copyright 2023 AIT - Austrian Institute of Technology
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import pandas as pd
import rclpy
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.node import Node
from builtin_interfaces.msg import Duration, Time
import sys
import signal
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance


class TrajectoryActionClient(Node):
    def __init__(self):
        super().__init__("trajectory_action_client")
        self.idx = 0
        self.declare_parameter("file_name", "src/ros2_control_demos/example_XX/swingup")
        self.declare_parameter("nrJoints", 2)
        self.declare_parameter("joint_names", ["slider_to_cart", "cart_to_pendulum"])
        self.declare_parameter("path_tolerance", [100.1, 100.1])
        self.declare_parameter("goal_tolerance", [0.05, 0.05])
        self.declare_parameter("map_joints", [0, 1])

        self.action_client_ = ActionClient(
            self, FollowJointTrajectory, "/trajectory_controllers/follow_joint_trajectory"
        )

        # safe handling of SIGINT/SIGTERM
        signal.signal(signal.SIGINT, self.safe_exit)
        signal.signal(signal.SIGTERM, self.safe_exit)

        # ---------------------------------------- read Excel-file ------------

        print("Load Excel-file")
        fileName = self.get_parameter("file_name").get_parameter_value().string_value
        fileExtension = ".csv"
        fileNameWithPath = fileName + fileExtension
        Data = pd.read_csv(
            fileNameWithPath,
            index_col=None,
            header=0,
            # usecols="A,B,C,D,E,F,G,H",
            # , header=None
        )
        self.Data_array = Data.values
        print("File name = %s" % (fileNameWithPath))  # output file name with logging

        # ------------------------- store data from Excel-file in variables ---
        self.nrJoints = self.get_parameter("nrJoints").value
        self.map_joints = (
            self.get_parameter("map_joints").get_parameter_value().integer_array_value
        )

        self.Data_array_size = len(self.Data_array[0, :])
        self.trajLen = len(self.Data_array[:, 0])
        print("Length of trajectory: " + str(self.trajLen))

        print("Data preview:")
        print(Data.head(2))
        sys.stdout.flush()

        self.joint_names_ = self.get_parameter("joint_names").value
        self.is_goal_rejected = False
        self.is_goal_accepted = False

        self.input_equals("\nStart the trajectory by pressing enter", "")
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names_
        goal_msg.trajectory.header.stamp = Time(sec=0, nanosec=0)  # start now

        # get points from csv data
        offset_idx = self.nrJoints
        for i_traj in range(self.trajLen):
            traj_point_message = JointTrajectoryPoint()
            traj_point_message.positions = [0.0] * self.nrJoints
            traj_point_message.velocities = [0.0] * self.nrJoints
            duration_in_seconds = self.Data_array[i_traj, 0]
            for i_pos in range(self.nrJoints):
                traj_point_message.positions[i_pos] = self.Data_array[
                    i_traj, 1 + self.map_joints[i_pos]
                ]
                traj_point_message.velocities[i_pos] = self.Data_array[
                    i_traj, 1 + offset_idx + self.map_joints[i_pos]
                ]

            traj_point_message.time_from_start = Duration(
                sec=int(duration_in_seconds),
                nanosec=int((duration_in_seconds - int(duration_in_seconds)) * 1e9),
            )
            goal_msg.trajectory.points.append(traj_point_message)

        # set goal and path tolerances for new goal
        # TODO: this does not work, see https://github.com/ros-controls/ros2_controllers/pull/716
        for i_pos in range(self.nrJoints):
            goal_tolerance = JointTolerance()
            goal_tolerance.name = self.joint_names_[i_pos]
            goal_tolerance.position = self.get_parameter("goal_tolerance").value[i_pos]
            goal_msg.goal_tolerance.append(goal_tolerance)
            path_tolerance = JointTolerance()
            path_tolerance.name = self.joint_names_[i_pos]
            path_tolerance.position = self.get_parameter("path_tolerance").value[i_pos]
            goal_msg.path_tolerance.append(path_tolerance)

        self.action_client_.wait_for_server()
        self._send_goal_future = self.action_client_.send_goal_async(
            goal_msg, self.get_feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info("Goal rejected!")
            self.is_goal_rejected = True
            return

        self.is_goal_accepted = True
        self.get_logger().info("Goal accepted!")

        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        print(result.error_code)
        print(result.error_string)
        if result.error_code == result.SUCCESSFUL:
            print("Trajectory goal reached!")

    def get_feedback_callback(self, feedback):

        # print(feedback.feedback.desired)
        # print(feedback.feedback.actual)
        # print(feedback.feedback.error)
        # print("Got feedback!")
        feedback  # eat warnings

    def safe_exit(self, *args):
        # stop when the program is terminated from cmdline
        if self.is_goal_accepted:
            print("stop action")
            future = self.goal_handle.cancel_goal_async()
            future.add_done_callback(self.goal_canceled_callback)

        rclpy.shutdown()

    def goal_canceled_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info("Cancelling of goal complete")
        else:
            self.get_logger().warning("Goal failed to cancel")

    def input_equals(self, message, input_str):
        text = "input"
        while not text == input_str:
            text = input(message)


def main(args=None):
    rclpy.init(args=args)

    trajectory_action_client = TrajectoryActionClient()

    try:
        rclpy.spin(trajectory_action_client)
    finally:
        trajectory_action_client.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
