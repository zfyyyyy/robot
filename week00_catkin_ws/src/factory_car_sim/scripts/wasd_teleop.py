#!/usr/bin/env python3
import sys, termios, tty, select
import rospy
from geometry_msgs.msg import Twist

def get_key(timeout=0.1):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if r else ""
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

if __name__ == "__main__":
    rospy.init_node("wasd_teleop")
    pub_ns = rospy.Publisher("/car1/cmd_vel", Twist, queue_size=10)
    pub_root = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    print("WASD control: w/s/a/d, space stop, q quit")

    while not rospy.is_shutdown():
        key = get_key()
        msg = Twist()
        if key == "w": msg.linear.x = 0.4
        elif key == "s": msg.linear.x = -0.4
        elif key == "a": msg.angular.z = 1.0
        elif key == "d": msg.angular.z = -1.0
        elif key == "q": break
        pub_ns.publish(msg)
        pub_root.publish(msg)
