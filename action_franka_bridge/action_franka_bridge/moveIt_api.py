"""
Utilizes Moveit2 to perform actions on the Franka robot. It launches
a moveit2 client node which contains the following ROS mechanisms:

SUBSCRIBERS:
  + /joint_states (JointState) - Listens to the current joint of the franka.
SERVICE CLIENTS:
  + /compute_cartesian_path (GetCartesianPath) - Computes a cartesian path from a series of waypoints.
ACTION CLIENTS:
  + /execute_trajectory (ExecuteTrajectory) - Performs the computed cartesian path.
"""

import rclpy
import rclpy.node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from moveit_msgs.msg import (
    RobotState,
    RobotTrajectory,
)
from geometry_msgs.msg import Pose

from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory

import threading
import random


class Moveit2Python:
    def __init__(self, base_frame, ee_frame, group_name):
        self.node = rclpy.create_node(f"moveit2_cli_node_{random.randint(0, 1000)}")
        self.base_frame_id = base_frame
        self.ee_frame_id = ee_frame
        self.group_name = group_name
        self.callback_group = ReentrantCallbackGroup()
        self.executor = MultiThreadedExecutor()
        self.robot_state = RobotState()

        self.sub_joint_states = self.node.create_subscription(
            JointState,
            "joint_states",
            self.sub_joint_state_callback,
            10,
            callback_group=self.callback_group,
        )

        self.cli_get_cartesian_path = self.node.create_client(
            GetCartesianPath,
            "compute_cartesian_path",
            callback_group=self.callback_group,
        )

        self.cli_action_execute_trajectory = ActionClient(
            self.node,
            ExecuteTrajectory,
            "execute_trajectory",
            callback_group=self.callback_group,
        )

        while not self.cli_get_cartesian_path.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().warn(
                f"Service {self.cli_get_cartesian_path.srv_name} not available, waiting again ..."
            )

        while self.node.count_publishers("joint_states") < 1:
            self.node.get_logger().warn(
                "Not enoutgh joint state message received, waiting again ..."
            )

        self.node.get_logger().info("API initialized")

        self.thread_node = threading.Thread(target=self.spin_node, daemon=True)
        self.thread_node.start()

    def __del__(self):
        self.node.get_logger().info("Shutting down api node")
        self.thread_node.join()
        self.node.destroy_node()
        rclpy.try_shutdown()

    def spin_node(self):
        if not rclpy.ok():
            rclpy.init(args=None)
        rclpy.spin(node=self.node, executor=self.executor)

    def sub_joint_state_callback(self, msg: JointState):
        self.robot_state.joint_state = msg

    async def get_cartesian_path(self, waypoints: list[Pose]):
        self.node.get_logger().info("Sending request to get cartesian path")
        request = GetCartesianPath.Request()
        request.header.stamp = self.node.get_clock().now().to_msg()
        request.header.frame_id = self.base_frame_id
        request.start_state = self.robot_state
        request.group_name = self.group_name
        request.link_name = self.ee_frame_id
        request.waypoints = waypoints

        request.max_step = 0.01
        request.avoid_collisions = True
        request.max_cartesian_speed = 0.08
        request.max_velocity_scaling_factor = 0.08
        request.max_acceleration_scaling_factor = 0.03
        request.cartesian_speed_limited_link = self.ee_frame_id

        future = self.cli_get_cartesian_path.call_async(request)
        await future

        result = future.result()
        self.node.get_logger().info("Get cartesian path finished")
        return result.solution

    async def execute_trajectory(self, trajectory: RobotTrajectory):
        self.node.get_logger().info("Sending request to execute trajectory")
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        future = self.cli_action_execute_trajectory.send_goal_async(goal)
        await future

        future = future.result().get_result_async()
        await future

        result = future.result().result
        self.node.get_logger().info("Executing trajectory finished")
        return result