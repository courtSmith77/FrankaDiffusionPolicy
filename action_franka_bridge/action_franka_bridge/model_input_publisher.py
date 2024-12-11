"""
Interpret command mode from user, and converts actions from the diffusion
model into robot motion.

This node interprets three command states from the user: Begin ('b'), Action
('a'), Pause ('p' or 's'). These states are used to control whether or not the
robot performs the actions from the subscribed /predicted_action topic. Actions
and current robot pose are used within two PD loops, one for position and one for
orientation, to control the robot.

SUBSCRIBERS:
  + /predicted_action (Pose) - The next action position from the diffusion model.
  + /command_mode (String) - The command mode based on the key pressed.
PUBLISHERS:
  + /text_marker (Marker) - The text marker that is published to RViz.
  + /bounding_box (Marker) - The bounding box marker that is published to RViz.
  + /desired_ee_pose (Pose) - The desired pose of the end effector.
SERVICE CLIENTS:
  + /robot_waypoints (PlanPath) - The service that plans and executes the robot's
    motion.
  + /record (Empty) - The service that initiates recording the demonstration data.
"""
from geometry_msgs.msg import Pose

from sensor_msgs.msg import Image

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_ros

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

import numpy as np
import cv2
from cv_bridge import CvBridge

class ModelInputPublisher(Node):

    def __init__(self):
        super().__init__('model_input_publisher')

        # frequency parameter
        self.declare_parameter('frequency', 10.0, ParameterDescriptor(description='Frequency (hz) of the timer callback'))
        self.timer_freqency = self.get_parameter('frequency').get_parameter_value().double_value

        # create subscribers
        self.scene_img_sub = self.create_subscription(Image, '/d435/color/image_raw', self.scene_image_callback, 10)
        self.ee_img_sub = self.create_subscription(Image, '/d405/color/image_rect_raw',  self.ee_image_callback, 10)

        # create publishers
        self.desired_ee_pub = self.create_publisher(Pose, 'desired_ee_pose', 10)
        self.scene_img_pub = self.create_publisher(Image, 'scene_image_obs', 10)
        self.ee_img_pub = self.create_publisher(Image, 'ee_image_obs', 10)

        # create timer
        self.timer = self.create_timer((1.0/self.timer_freqency), self.timer_callback)

        # create tf buffer and listener
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.bridge = CvBridge()

        self.current_scene_img = None
        self.current_ee_img = None
        self.scene_flag = False
        self.ee_flag = False

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
        ee_home_pos, ee_home_rot = self.get_transform("panda_link0", "panda_hand_tcp")
        ee_pose = Pose()
        ee_pose.position.x = ee_home_pos.x
        ee_pose.position.y = ee_home_pos.y
        ee_pose.position.z = ee_home_pos.z
        ee_pose.orientation.x = ee_home_rot.x
        ee_pose.orientation.y = ee_home_rot.y
        ee_pose.orientation.z = ee_home_rot.z
        ee_pose.orientation.w = ee_home_rot.w
        return ee_pose
    
    def scene_image_callback(self, msg):
        """Get the current scene image from the realsense."""

        # TODO: make image size not hard coded
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # old cropping
        # crop_img = img[150:720, 200:1080]
        # small_img = cv2.resize(crop_img, (110,70))
        # new cropping
        crop_img = img[340:710, 300:800]
        small_img = cv2.resize(crop_img, (165,125))

        self.current_scene_img = small_img
        self.scene_flag = True

    def ee_image_callback(self, msg):
        """Get the current ee image from the realsense."""

        # TODO: make image size not hard coded
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        small_img = cv2.resize(img, (212,120))

        self.current_ee_img = small_img
        self.ee_flag = True

    def timer_callback(self):
        """Callback for the timer."""

        # publish actual ee pose for diffusion and data collect
        try:
            ee_pose = self.get_ee_pose()
            self.desired_ee_pub.publish(ee_pose)
        except AttributeError as e:
            return

        # publish transformed scene image
        if self.current_scene_img is not None:
            scene_img_msg = self.bridge.cv2_to_imgmsg(self.current_scene_img, encoding='bgr8')
            self.scene_img_pub.publish(scene_img_msg)

        # publish transformed scene image
        if self.current_ee_img is not None:
            ee_img_msg = self.bridge.cv2_to_imgmsg(self.current_ee_img, encoding='bgr8')
            self.ee_img_pub.publish(ee_img_msg)


def main(args=None):
    rclpy.init(args=args)
    model_input_publisher = ModelInputPublisher()
    rclpy.spin(model_input_publisher)

if __name__ == '__main__':
    main()




