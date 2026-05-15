#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist


class CmdVelFollower:
    def __init__(self):
        leader_topic = rospy.get_param("~leader_topic", "/car1/cmd_vel")
        follower_topics = rospy.get_param("~follower_topics", ["/car2/cmd_vel", "/car3/cmd_vel"])
        self.publishers = [rospy.Publisher(topic, Twist, queue_size=10) for topic in follower_topics]
        rospy.Subscriber(leader_topic, Twist, self.cb, queue_size=10)
        rospy.loginfo("cmd_vel_follower started: leader=%s followers=%s", leader_topic, follower_topics)

    def cb(self, msg):
        for pub in self.publishers:
            pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("cmd_vel_follower")
    CmdVelFollower()
    rospy.spin()