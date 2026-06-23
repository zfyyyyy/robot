#!/usr/bin/env python3

import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2


class ThinLiveCloudPublisher:
    def __init__(self):
        self.stride = max(1, rospy.get_param("~stride", 2))
        self.publisher = rospy.Publisher(
            "/cloud_registered_display", PointCloud2, queue_size=2
        )
        self.subscriber = rospy.Subscriber(
            "/cloud_registered", PointCloud2, self.on_cloud, queue_size=1
        )

    def on_cloud(self, message):
        points = []
        for index, point in enumerate(
            point_cloud2.read_points(
                message, field_names=("x", "y", "z"), skip_nans=True
            )
        ):
            if index % self.stride == 0:
                points.append(point)

        self.publisher.publish(point_cloud2.create_cloud_xyz32(message.header, points))


if __name__ == "__main__":
    rospy.init_node("thin_live_cloud")
    ThinLiveCloudPublisher()
    rospy.spin()
