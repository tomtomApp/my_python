#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

class KeyTeleop(Node):
    cmd_bindings = {'q': [1, 1],
                    'w': [1, 0],
                    'e': [1, -1],
                    'a': [0, 1],
                    'd': [0, -1],
                    'z': [-1, -1],
                    'x': [-1, 0],
                    'c': [-1, 1]}
    
    set_bindings = {'t': [1, 1],
                    'b': [-1, -1],
                    'y': [1, 0],
                    'n': [-1, 0],
                    'u': [0, 1],
                    'm': [0, -1]}
    
    def __init__(self):
        super().__init__('keyboard_teleop')
        # Save terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        # Initial values
        self.inc_ratio = 0.1
        self.speed = [0.25, 0.5]  # Initial speeds
        self.command = [0, 0]
        self.update_rate = 10  # Hz
        self.alive = True
        # Setup publisher
        self.pub_twist = self.create_publisher(Twist, '/cmd_vel_keyboard', 10)
        self.timer = self.create_timer(1.0 / self.update_rate, self.timer_callback)
    
    def fini(self):
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    
    def timer_callback(self):
        ch = self.get_key()
        self.process_key(ch)
        self.update()
        self.show_status()

    def print_usage(self):
        msg = """
        Keyboard Teleop that Publish to /cmd_vel_keyboard (geometry_msgs/Twist)
        Copyright (C) 2024
        Released under BSD License
        --------------------------------------------------
        H:       Print this menu
        Moving around:
          Q   W   E
          A   S   D
          Z   X   C
        T/B :   increase/decrease max speeds 10%
        Y/N :   increase/decrease only linear speed 10%
        U/M :   increase/decrease only angular speed 10%
        anything else : stop

        G :   Quit
        --------------------------------------------------
        """
        self.loginfo(msg)

    def loginfo(self, msg):
        # Used to print items to screen
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        print(msg)
        tty.setraw(sys.stdin.fileno())

    def show_status(self):
        msg = 'Status:\tlinear %.2f\tangular %.2f' % (self.speed[0], self.speed[1])
        self.loginfo(msg)

    def process_key(self, ch):
        if ch == 'h':
            self.print_usage()
        elif ch in self.cmd_bindings.keys():
            self.command = self.cmd_bindings[ch]
            self.loginfo('pushed_key')
        elif ch in self.set_bindings.keys():
            self.speed = [s * (1 + self.set_bindings[ch][i] * self.inc_ratio) for i, s in enumerate(self.speed)]
        elif ch == 'g':
            self.loginfo('Quitting')
        else:
            self.command = [0, 0]

    def update(self):
        twist = Twist()
        twist.linear.x = self.speed[0] * self.command[0]
        twist.angular.z = self.speed[1] * self.command[1]
        self.pub_twist.publish(twist)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        return key.lower()

def main(args=None):
    rclpy.init(args=args)
    teleop = None
    try:
        teleop = KeyTeleop()
        teleop.print_usage()
        rclpy.spin(teleop)
    except KeyboardInterrupt:
        teleop.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        if teleop:
            teleop.fini()
            teleop.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
