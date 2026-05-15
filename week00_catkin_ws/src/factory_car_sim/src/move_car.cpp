#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "move_car_node");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/car1/cmd_vel", 10);
  ros::Rate rate(10);

  while (ros::ok()) {
    geometry_msgs::Twist msg;
    pub.publish(msg);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
