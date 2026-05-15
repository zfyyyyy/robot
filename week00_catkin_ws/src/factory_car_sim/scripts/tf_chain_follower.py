#!/usr/bin/env python3
import math
import rospy
import tf2_ros
from geometry_msgs.msg import Twist

class TfChainFollower:
    def __init__(self):
        self.leader_frame = rospy.get_param("~leader_frame")
        self.follower_frame = rospy.get_param("~follower_frame")
        self.cmd_topic = rospy.get_param("~cmd_topic")

        self.linear_gain = rospy.get_param("~linear_gain", 1.0)
        self.angular_gain = rospy.get_param("~angular_gain", 3.0)
        self.max_linear = rospy.get_param("~max_linear", 1.2)
        self.max_angular = rospy.get_param("~max_angular", 2.0)
        self.stop_distance = rospy.get_param("~stop_distance", 0.25)

        self.pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            cmd = Twist()
            try:
                trans = self.buffer.lookup_transform(
                    self.follower_frame, self.leader_frame,
                    rospy.Time(0), rospy.Duration(0.2)
                )
                dx = trans.transform.translation.x
                dy = trans.transform.translation.y
                dist = math.sqrt(dx * dx + dy * dy)

                if dist > self.stop_distance:
                    cmd.linear.x = min(self.linear_gain * dist, self.max_linear)
                    ang = self.angular_gain * math.atan2(dy, dx)
                    cmd.angular.z = max(-self.max_angular, min(ang, self.max_angular))
            except Exception:
                pass

            self.pub.publish(cmd)
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("tf_chain_follower")
    TfChainFollower().run()
