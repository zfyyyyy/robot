#!/usr/bin/env python3
import rospy
import tf2_ros
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped

def cb(msg):
    now = rospy.Time.now()
    m = {n:i for i,n in enumerate(msg.name)}
    br = cb.br
    for name, frame in [("car1","car1_world"),("car2","car2_world"),("car3","car3_world")]:
        if name not in m:
            continue
        p = msg.pose[m[name]]
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "map"
        t.child_frame_id = frame
        t.transform.translation.x = p.position.x
        t.transform.translation.y = p.position.y
        t.transform.translation.z = p.position.z
        t.transform.rotation = p.orientation
        br.sendTransform(t)

if __name__ == "__main__":
    rospy.init_node("model_tf_broadcaster")
    cb.br = tf2_ros.TransformBroadcaster()
    rospy.Subscriber("/gazebo/model_states", ModelStates, cb, queue_size=1)
    rospy.spin()
