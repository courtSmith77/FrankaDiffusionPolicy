"""
Collect data for visualizing inference actions, executed actions,
and the end effector position. All data is saved to a "./data/" folder in individual csv files.

PUBLISHERS:
  + /predicted_action (Float32MultiArray) - The actions to be executed directly from the diffusion node.
  + /action_horizon (Float32MultiArray) - The full action horizon directly from the diffusion node.
  + /current_action (Float32MultiArray) - The action sequence about to be executed on the robot.
  + /ee_before_action (Float32MultiArray) - The end effector position before executing each action sequence.
  + /current_ee_pose (Float32MultiArray) - The end effector position at all times sampled from the tf tree.
SERVICES:
  + /record (Empty) - Enables saving the image and scene data
"""
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty

from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import csv
import numpy as np
from datetime import datetime

class DataCollection(Node):

    def __init__(self):
        super().__init__('data_collection')

        # subscriptions for actions
        self.predicted_action_sub = self.create_subscription(Float32MultiArray, '/predicted_action', self.predicted_action_callback, 10)
        self.action_horizon_sub = self.create_subscription(Float32MultiArray, '/action_horizon', self.action_horizon_callback, 10)
        self.current_action_sub = self.create_subscription(Float32MultiArray, '/current_action', self.current_action_callback, 10)
        self.ee_at_action_sub = self.create_subscription(Float32MultiArray, '/ee_before_action', self.ee_before_action_callback, 10)
        self.ee_at_all_time_sub = self.create_subscription(Pose, '/current_ee_pose', self.ee_all_time_callback, 10)

        # create service
        self.record_srv = self.create_service(Empty, '/record', self.record_callback)

        # create timer
        self.timer = self.create_timer(0.01, self.timer_callback)

        self.bridge = CvBridge()

        # create tf buffer and listener
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.received_ee_pose = False
        self.received_ee_image = False
        self.received_scene_image = False
        self.start_recording = False

        self.count = 0
        self.end = True

    def predicted_action_callback(self, msg):
        """Callback for saving the predicted action sequences."""
        self.get_logger().info('Recieved Predicted Action')
        if self.start_recording:
            pa_arr = msg.data
            rows = msg.layout.dim[0].size
            cols = msg.layout.dim[1].size

            with open('./data/predicted_action.csv', mode='a') as self.pa_file:
                self.pa_csv_writer = csv.writer(self.pa_file)
                pa = np.array(pa_arr).reshape((rows,cols))
                curr_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
                time_column = np.full((pa.shape[0], 1), curr_time)
                pa_with_time = np.hstack((time_column, pa))
                self.pa_csv_writer.writerows(pa_with_time)
            
            self.get_logger().info('Saved Predicted Action')

    def action_horizon_callback(self, msg):
        """Callback for saving the entire action horizon sequences."""
        if self.start_recording:
            ah_arr = msg.data
            rows = msg.layout.dim[0].size
            cols = msg.layout.dim[1].size

            with open('./data/action_horizon.csv', mode='a') as self.ah_file:
                self.ah_csv_writer = csv.writer(self.ah_file)
                ah = np.array(ah_arr).reshape((rows,cols))
                curr_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
                time_column = np.full((ah.shape[0], 1), curr_time)
                ah_with_time = np.hstack((time_column, ah))
                self.ah_csv_writer.writerows(ah_with_time)
    
    def current_action_callback(self, msg):
        """Callback for saving the current action about to be executed."""
        if self.start_recording:
            ca_arr = list(msg.data)
            rows = msg.layout.dim[0].size
            cols = msg.layout.dim[1].size

            with open('./data/current_action.csv', mode='a') as self.ca_file:
                self.ca_csv_writer = csv.writer(self.ca_file)
                ca = np.array(ca_arr).reshape((rows,cols))
                curr_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
                time_column = np.full((ca.shape[0], 1), curr_time)
                ca_with_time = np.hstack((time_column, ca))
                self.ca_csv_writer.writerows(ca_with_time)

    def ee_before_action_callback(self, msg):
        """Callback for saving the ee pose before executing the current action."""
        if self.start_recording:
            eb_arr = list(msg.data)

            with open('./data/ee_before_action.csv', mode='a') as self.eb_file:
                self.eb_csv_writer = csv.writer(self.eb_file)
                curr_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
                eb_with_time = [curr_time] + eb_arr
                self.eb_csv_writer.writerow(eb_with_time)
                self.get_logger().info('Saved to EE before csv')

    def ee_all_time_callback(self, msg):
        """Callback for saving the ee position at all times."""
        if self.start_recording:
            eat_arr = [msg.position.x, msg.position.y]

            with open('./data/ee_all_time.csv', mode='a') as self.eat_file:
                self.eat_csv_writer = csv.writer(self.eat_file)
                curr_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
                eat_with_time = [curr_time] + eat_arr
                self.eat_csv_writer.writerow(eat_with_time)

    def record_callback(self, request, response):
        """Callback for the start recording callback"""
        self.start_recording = True
        self.get_logger().info('Starting to record...')
        return response
    
    def timer_callback(self):
        """Callback for the timer."""

        if self.start_recording:

            try:
                trans = self.buffer.lookup_transform("panda_link0", "panda_hand_tcp", rclpy.time.Time())
                translation = trans.transform.translation
                at_arr = [translation.x, translation.y]

                with open('./data/ee_all_time_100hz.csv', mode='a') as self.at_file:
                    self.at_csv_writer = csv.writer(self.at_file)
                    curr_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
                    at_with_time = [curr_time] + at_arr
                    self.at_csv_writer.writerow(at_with_time)

            except:
                self.get_logger().info('Could not get ee pose transform')

def main(args=None):
    rclpy.init(args=args)
    data_collection = DataCollection()
    rclpy.spin(data_collection)

if __name__ == '__main__':
    main()















