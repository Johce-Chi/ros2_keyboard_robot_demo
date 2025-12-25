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

        # 修改 1: 发布到标准话题 /cmd_vel，配合 Launch 文件的 remappings 使用
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.publish_cmd)  # 10Hz

        self.current_cmd = Twist()
        self.settings = termios.tcgetattr(sys.stdin)

        self.get_logger().info('--- 键盘控制已启动 ---')
        self.get_logger().info('控制方式: W-前, S-后, A-左转, D-右转, 空格-停止, Q-退出')

    def get_key(self):
        # 实时监听键盘
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
            self.current_cmd.linear.x = 0.8  # 稍微提速
            self.current_cmd.angular.z = 0.0
        elif key == 's':
            self.current_cmd.linear.x = -0.8
            self.current_cmd.angular.z = 0.0
        elif key == 'a':
            self.current_cmd.angular.z = 1.0
            # 如果你想原地旋转，就把 linear.x 设为 0
        elif key == 'd':
            self.current_cmd.angular.z = -1.0
        elif key == ' ':
            self.current_cmd = Twist()
        elif key == 'q':
            # 修改 2: 退出前务必恢复终端属性，否则终端会乱码
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            self.get_logger().info('正在退出...')
            sys.exit(0)
        
        # 注意：如果你注释掉下面这两行，小车就会保持上一次的运动状态，直到你按新键
        elif key is None:
            # 保持当前状态还是停车？如果你想手感好，建议注释掉下面这行
             self.current_cmd = Twist() 

        self.pub.publish(self.current_cmd)

def main():
    rclpy.init()
    try:
        node = KeyboardCar()
        rclpy.spin(node)
    except SystemExit:
        pass
    except Exception as e:
        print(e)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
