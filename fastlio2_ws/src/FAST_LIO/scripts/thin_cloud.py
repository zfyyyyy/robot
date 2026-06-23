#!/usr/bin/env python3

import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2


class ThinCloudPublisher:
    def __init__(self):
        self.stride = max(1, rospy.get_param("~stride", 5))
        self.publisher = rospy.Publisher(
            "/cloud_pcd_display", PointCloud2, queue_size=1, latch=True
        )
        self.subscriber = rospy.Subscriber(
            "/cloud_pcd", PointCloud2, self.on_cloud, queue_size=1
        )
        self.published = False

    def on_cloud(self, message):
        if self.published:
            return

        points = []
        for index, point in enumerate(
            point_cloud2.read_points(
                message, field_names=("x", "y", "z"), skip_nans=True
            )
        ):
            if index % self.stride == 0:
                points.append(point)

        output = point_cloud2.create_cloud_xyz32(message.header, points)
        self.publisher.publish(output)
        self.published = True
        rospy.loginfo(
            "Published %d lightweight map points on /cloud_pcd_display",
            len(points),
        )
        self.subscriber.unregister()


if __name__ == "__main__":
    rospy.init_node("thin_pcd_cloud")
    ThinCloudPublisher()
    rospy.spin()
