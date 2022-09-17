import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

import math

class RemoconPS5(Node):

    def __init__(self):

        super().__init__('remocon_ps5')

        self.declare_parameter('max_trans', 0.5)
        self.declare_parameter('max_rot', math.pi)

        self.max_trans = self.get_parameter('max_trans').value
        self.max_rot = self.get_parameter('max_rot').value

        self.add_on_set_parameters_callback(self.parameters_callback)

        self.cmd_vel_pub_ = self.create_publisher(
            Twist,
            '/cmd_vel_joy',
            10)

        self.joy_sub_ = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)

        self.high_or_slow = False

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'max_trans':
                self.max_trans = param.value
            elif param.name == 'max_rot':
                self.max_rot = param.value
            else:
                raise ValueError(f'unknown parameer {param.name}')
        return SetParametersResult(successful=True)


    def joy_callback(self, msg: Joy):
        
        cmd_vel = Twist()

        if msg.buttons[4]:
            if self.high_or_slow:
                self.high_or_slow = False
            else:
                self.high_or_slow = True


        trans_vel = - self.max_trans * msg.axes[1]
        rot_vel = - self.max_rot * msg.axes[0]

        if self.high_or_slow:
            trans_vel = 0.5 * trans_vel
            rot_vel = 0.5 * rot_vel

        cmd_vel.linear.x = trans_vel
        cmd_vel.angular.z = rot_vel

        self.cmd_vel_pub_.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)

    remocon = RemoconPS5()

    rclpy.spin(remocon)

    remocon.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()