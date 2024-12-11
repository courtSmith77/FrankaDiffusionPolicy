"""
Control the status of the diffusion model inference and action deployment. It
launches a xterm screen where key presses indicate:
    'b' - "begin" enable diffusion
    'p' - "pause" disable diffusion

SERVICE CLIENTS:
  + /enable_diffusion (Empty) - Enables diffusion with an empty request.
  + /disable_diffusion (Empty) - Disables diffusion with an empty request.
  + /record (Empty) - Triggers recording data through the data_collection node.
"""
import rclpy
from rclpy.node import Node

from std_srvs.srv import Empty

import sys
import tty
import termios
from select import select


class CommandMode(Node):
    def __init__(self):
        super().__init__("command_mode")

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
    command_mode = CommandMode()
    rclpy.spin(command_mode)

if __name__ == '__main__':
    main()
