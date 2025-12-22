import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select


class KeyboardCar(Node):
    def __init__(self):
        super().__init__('keyboard_car')

        self.pub = self.create_publisher(Twist, '/model/vehicle_blue/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.publish_cmd)  # 10Hz

        self.current_cmd = Twist()

        self.settings = termios.tcgetattr(sys.stdin)

        self.get_logger().info('Keyboard car controller started (WASD, q quit)')

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = None
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def publish_cmd(self):
        key = self.get_key()

        if key == 'w':
            self.current_cmd.linear.x = 0.5
            self.current_cmd.angular.z = 0.0
        elif key == 's':
            self.current_cmd.linear.x = -0.5
            self.current_cmd.angular.z = 0.0
        elif key == 'a':
            self.current_cmd.angular.z = 1.0
        elif key == 'd':
            self.current_cmd.angular.z = -1.0
        elif key == ' ':
            self.current_cmd = Twist()
        elif key == 'q':
            rclpy.shutdown()
            return
        elif key is None:
            # 没按键 → 停车
            self.current_cmd = Twist()

        self.pub.publish(self.current_cmd)


def main():
    rclpy.init()
    node = KeyboardCar()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

