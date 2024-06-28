#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
import curses
from threading import Timer

class BumperController():
    def __init__(self, window):
        self.window = window
        self.bumper_triggered = False
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.bumper_sub = rospy.Subscriber('bumper', Bool, self.get_bumper, queue_size=10)
        self.twist = Twist()
        self.key_timer = None
        self.control_robot()

    def handle_key_input(self, key):
        if not self.bumper_triggered:
            if key == ord('w'):
                self.twist.linear.x = 1.0
            elif key == ord('s'):
                self.twist.linear.x = -1.0
            elif key == ord('a'):
                self.twist.angular.z = 1.0
            elif key == ord('d'):
                self.twist.angular.z = -1.0
            else:
                self.twist.linear.x = 0
                self.twist.angular.z = 0
                if self.key_timer:
                    self.key_timer.cancel()

            self.cmd_pub.publish(self.twist)

    def get_bumper(self, bumper_msg):
        self.bumper_triggered = bumper_msg.data

        if self.bumper_triggered:
            rospy.signal_shutdown("Bumper Acionado - Comunicação Cortada")

    def key_timer_callback(self):
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.cmd_pub.publish(self.twist)

    def control_robot(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            key = self.window.getch()
            self.handle_key_input(key)

            if key in [ord('w'), ord('s'), ord('a'), ord('d')]:
                if self.key_timer:
                    self.key_timer.cancel()
                self.key_timer = Timer(2.0, self.key_timer_callback)
                self.key_timer.start()

            # Publicar informações de odometria diretamente no terminal
            try:
                odom_msg = rospy.wait_for_message('odom', Odometry, timeout=1.0)
                position = odom_msg.pose.pose.position
                orientation = odom_msg.pose.pose.orientation
                print(f"Posição: x={position.x}, y={position.y}, z={position.z}")
                print(f"Orientação: x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}")
            except rospy.ROSException:
                pass

            rate.sleep()

def main():
    rospy.init_node('robotino_controller')
    window = curses.initscr()
    curses.cbreak()
    window.keypad(True)

    try:
        bumper_controller = BumperController(window)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        curses.nocbreak()
        window.keypad(False)
        curses.endwin()

if __name__ == '__main__':
    main()
