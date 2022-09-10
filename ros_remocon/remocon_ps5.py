import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class RemoconPS5(Node):

    def __init__(self):

        super().__init__('remocon_ps5')

        self.cmd_vel_pub_ = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

        self.joy_sub_ = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)

    def joy_callback(self, msg: Joy):
        
        cmd_vel = Twist()

        turn_vel = - msg.axes[0]
        foward_vel = - msg.axes[1]

        cmd_vel.linear.x = foward_vel
        cmd_vel.angular.z = turn_vel

        self.cmd_vel_pub_.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)

    remocon = RemoconPS5()

    rclpy.spin(remocon)

    remocon.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()