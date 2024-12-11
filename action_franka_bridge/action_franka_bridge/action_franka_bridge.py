"""
Receive the action sequence from the diffusion inference, convert it into
a sequence of waypoints, and request the actions on the robot using a
Franka MoveIt API.

PUBLISHERS:
  + /current_action (Float32MultiArray) - The current action sequence that is being executed (for data collection only).
  + /ee_before_action (Float32MultiArray) - The end effector pose before the action sequence is executed (for data collection only).
SUBSCRIBERS:
  + /predicted_action (Float32MultiArray) - The next action sequence from the diffusion model.
SERVICE CLIENTS:
  + /start_diffusion (Empty) - The service that triggers the next diffusion inference after robot execution is complete.
"""
from geometry_msgs.msg import Pose, Point, Quaternion

from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from std_srvs.srv import Empty

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_ros

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rcl_interfaces.msg import ParameterDescriptor

import numpy as np

from action_franka_bridge.moveIt_api import Moveit2Python

class ActionFrankaBridge(Node):

    def __init__(self):
        super().__init__('action_franka_bridge')

        # instatiate moveIt wrapper
        self.api = Moveit2Python(
            base_frame="panda_link0",
            ee_frame="panda_hand_tcp",
            group_name="panda_manipulator"
            )

        # frequency parameter
        self.declare_parameter('frequency', 10.0, ParameterDescriptor(description='Frequency (hz) of the timer callback'))
        self.timer_freqency = self.get_parameter('frequency').get_parameter_value().double_value

        # create callback groups
        self.action_callback_group = MutuallyExclusiveCallbackGroup()

        # create subscribers
        self.action_subscriber = self.create_subscription(Float32MultiArray, 'predicted_action', self.action_callback, 10, callback_group=self.action_callback_group)

        # publishers for actions
        self.current_action_pub = self.create_publisher(Float32MultiArray, 'current_action', 10)
        self.ee_at_action_pub = self.create_publisher(Float32MultiArray, 'ee_before_action', 10)

        # create client for requesting inference
        self.diffusion_start_client = self.create_client(Empty, 'start_diffusion')
        self.diffusion_start_client.wait_for_service(timeout_sec=2.0)

        # create timer
        self.timer = self.create_timer((1.0/self.timer_freqency), self.timer_callback)

        # create tf buffer and listener
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        # action variables
        self.pending_action = False
        self.action_array = None
        self.waypoints = None

        self.x_limits = [0.15, 1.0]
        self.y_limits = [-0.75, 0.6]
        self.y_inner = [-0.15, 0.15]
        self.z_limits = [0.07, 0.75]

        self.count = 0

    def get_transform(self, target_frame, source_frame):
        """Get the transform between two frames."""
        try:
            trans = self.buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            translation = trans.transform.translation
            rotation = trans.transform.rotation
            return translation, rotation

        except tf2_ros.LookupException as e:
            # the frames don't exist yet
            self.get_logger().info(f"Lookup exception: {e}")
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]
        except tf2_ros.ConnectivityException as e:
            # the tf tree has a disconnection
            self.get_logger().info(f"Connectivity exception: {e}")
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]
        except tf2_ros.ExtrapolationException as e:
            # the times are two far apart to extrapolate
            self.get_logger().info(f"Extrapolation exception: {e}")
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]

    def get_ee_pose(self):
        """Get the current pose of the end-effector."""

        try:
            ee_home_pos, _ = self.get_transform("panda_link0", "panda_hand_tcp")
            return [ee_home_pos.x, ee_home_pos.y]
        except:
            self.get_logger().info('Could not get ee pose transform.')

    def generate_waypoints(self):
        """Generate waypoints and check for boundary limits."""

        self.waypoints = []

        for i, (x,y) in enumerate(self.action_array):

            if x < self.x_limits[0] or x > self.x_limits[1]:
                self.action_array[i][0] = self.x_limits[0] if x < self.x_limits[0] else self.x_limits[1]
                self.get_logger().info('Trying to go to far out of X')
            
            if (y < self.y_limits[0] or y > self.y_limits[1]):
                self.action_array[i][1] = self.y_limits[0] if self.desired_ee_pose.position.y < self.y_limits[0] else self.y_limits[1]
                self.get_logger().info('Trying to go to far out of Y')
            
            if ((y < self.y_inner[1] and y > self.y_inner[0]) and x < self.x_limits[0]):
                self.get_logger().info('Too close to base!!!!!!!!!!!!!')
                upper_diff = abs(self.y_inner[1] - y)
                lower_diff = abs(self.y_inner[0] - y)
                self.action_array[i][1] = self.y_inner[1] if upper_diff < lower_diff else self.y_inner[0]

            temp_pose = Pose(
                             position=Point(x=float(self.action_array[i][0]), y=float(self.action_array[i][1]), z=0.085),
                             orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0),
                            )
            
            self.waypoints.append(temp_pose)

    def action_callback(self, msg):
        """Callback for the action subscriber."""

        # only save action once previous action set is done executing
        if not self.pending_action:
            arr = msg.data
            rows = msg.layout.dim[0].size
            cols = msg.layout.dim[1].size

            self.action_array = np.array(arr).reshape((rows,cols))
            self.pending_action = True

            self.get_logger().info(f'Action pairs = {self.action_array}')

    async def timer_callback(self):
        """Callback for the timer."""
        
        if self.action_array is not None and self.pending_action:

            # Crop to bound area
            self.generate_waypoints()

            # publish actions
            action_msg = Float32MultiArray()
            action_msg.data = self.action_array.flatten().tolist()
            # add dimensions of flattened array
            action_msg.layout.dim.append(MultiArrayDimension())
            action_msg.layout.dim[0].label = "rows"
            action_msg.layout.dim[0].size = self.action_array.shape[0]
            action_msg.layout.dim[0].stride = self.action_array.shape[0] * self.action_array.shape[1]
            action_msg.layout.dim.append(MultiArrayDimension())
            action_msg.layout.dim[1].label = "columns"
            action_msg.layout.dim[1].size = self.action_array.shape[1]
            action_msg.layout.dim[1].stride = self.action_array.shape[1]
            self.current_action_pub.publish(action_msg)
            
            # publish ee position
            ee_pos = self.get_ee_pose()
            ee_msg = Float32MultiArray()
            ee_msg.data = ee_pos
            self.ee_at_action_pub.publish(ee_msg)

            self.get_logger().info('Sending Actions to Move it.')
            # perform get cartesian path
            trajectory = await self.api.get_cartesian_path(self.waypoints)

            # perform execute trajectory
            result = await self.api.execute_trajectory(trajectory)

            # set pending_action to False
            self.pending_action = False

            # call for another inference
            future = self.diffusion_start_client.call_async(Empty.Request())

        else:
            self.pending_action = False


def main(args=None):
    rclpy.init(args=args)
    action_franka_bridge = ActionFrankaBridge()
    rclpy.spin(action_franka_bridge)

if __name__ == '__main__':
    main()