#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import curses  # Importing curses for keyboard input

class BumperController():
    def __init__(self, window):
        self.window = window
        self.bumper_triggered = True
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.bumper_sub = rospy.Subscriber('bumper', Bool, self.get_bumper, queue_size=10)
        self.twist = Twist()
        self.control_robot()

    def handle_key_input(self, key):
        if key == ord('w'):
            self.twist.linear.x = 1.0  # Move forward
        elif key == ord('s'):
            self.twist.linear.x = -1.0  # Move backward
        elif key == ord('a'):
            self.twist.angular.z = 1.0  # Turn left
        elif key == ord('d'):
            self.twist.angular.z = -1.0  # Turn right
        else:
            self.twist.linear.x = 0
            self.twist.angular.z = 0  # Stop

        self.cmd_pub.publish(self.twist)

    def get_bumper(self, bumper_msg):
        # Bumper message handling
        self.bumper_triggered = bumper_msg.data
        print("Bumper state:", self.bumper_triggered)

        if self.bumper_triggered:
            print("Bumper Triggered - Stopping Robot")
            self.twist.linear.x = 0
            self.twist.angular.z = 0  # Stop            
            self.cmd_pub.publish(self.twist)
        else:
            print("Bumper Not Triggered")

    def control_robot(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            key = self.window.getch()
            self.handle_key_input(key)
            rate.sleep()

def main():
    rospy.init_node('robotino_bumper_controller')
    window = curses.initscr()
    curses.cbreak()
    window.keypad(True)
    try:
        bumper_controller = BumperController(window)
    except rospy.ROSInterruptException:
        pass
    finally:
        curses.nocbreak()
        window.keypad(False)
        curses.endwin()

if __name__ == '__main__':
    main()
