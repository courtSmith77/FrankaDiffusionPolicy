"""
Control the status of the diffusion model inference and action deployment.

Using tty and sys to read keyboard inputs to trigger service calls to start
or stop the diffusion model inference and action deployment within the 
diffusion_policy node. Additionally, the command_mode is sent to the TODO: input node name
to trigger state updates.

PUBLISHERS:
  + /command_mode (String) - The command mode for the key pressed.
SERVICE CLIENTS:
  + /start_inference (Empty) - Enables inference with empty request
  + /start_action (Empty) - Enables action deployment with empty request
  + /stop_inference (Empty) - Disables inference with empty request
  + /stop_action (Empty) - Disables inference with empty request
"""
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_msgs.msg import String
from std_srvs.srv import Empty

import sys
import tty
import termios
from select import select


class CommandMode(Node):
    def __init__(self):
        super().__init__("commandmode")

        # create timer
        self.timer = self.create_timer(0.01, self.timer_callback)

        # service clients
        self.enabled_diffusion_client = self.create_client(Empty, 'enable_diffusion')
        self.enabled_diffusion_client.wait_for_service(timeout_sec=2.0)

        self.disabled_diffusion_client = self.create_client(Empty, 'disable_diffusion')
        self.disabled_diffusion_client.wait_for_service(timeout_sec=2.0)

        self.record_client = self.create_client(Empty, 'record')
        self.record_client.wait_for_service(timeout_sec=2.0)

        # keyboard hot keys
        self.get_logger().info("Press the letter 'b' to enable diffusion.\n")
        self.get_logger().info("Press the letter 'p' to disable diffusion.\n")
        self.settings = termios.tcgetattr(sys.stdin)
        self.timeout = 0.01

    def getKey(self):
        """Read keyboard inputs from the terminal or extern window."""
        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select([sys.stdin], [], [], self.timeout)
            if rlist:
                self.key = sys.stdin.read(1)
            else:
                self.key = None
        except Exception as e:
            self.get_logger().info(f'ERROR: {e}')
        
    def check_keys(self):
        """Check key press for hot keys and performs associated action."""
        if self.key == 'b':

            self.get_logger().info(f'Starting diffusion inference now.\r\n')
            future = self.enabled_diffusion_client.call_async(Empty.Request())
            future = self.record_client.call_async(Empty.Request())

        elif self.key == 'p':

            self.get_logger().info(f'Stopping diffusion inference now.\r\n')
            future = self.disabled_diffusion_client.call_async(Empty.Request())

    def timer_callback(self):
        """Publish the command mode."""

        self.getKey()
        self.check_keys()


def main(args=None):
    rclpy.init(args=args)
    commandmode = CommandMode()
    rclpy.spin(commandmode)

if __name__ == '__main__':
    main()
