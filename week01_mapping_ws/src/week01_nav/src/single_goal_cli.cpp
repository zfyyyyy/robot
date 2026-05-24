#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <cstdlib>
#include <cmath>

int main(int argc, char** argv) {
  // 初始化 ROS 节点，节点名为 single_goal_cli。
  ros::init(argc, argv, "single_goal_cli");

  // 命令行必须传入目标点：x y yaw(rad)，例如 rosrun week01_nav single_goal_cli 1.0 0.5 1.57。
  if (argc != 4) {
    ROS_ERROR("Usage: rosrun week01_nav single_goal_cli x y yaw(rad)");
    return 1;
  }

  // atof 将字符串参数转为 double；yaw 使用弧度制。
  double x = atof(argv[1]), y = atof(argv[2]), yaw = atof(argv[3]);

  // 连接 move_base action server，后续通过它发送导航目标。
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
  ROS_INFO("Waiting for move_base (30s)...");
  if (!ac.waitForServer(ros::Duration(30.0))) {
    ROS_ERROR("move_base not available.");
    return 3;
  }

  move_base_msgs::MoveBaseGoal goal;

  // 目标点使用 map 坐标系；导航时要保证 map、odom、base_link/base_footprint 的 TF 正常。
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;

  // 平面机器人只绕 z 轴旋转，因此由 yaw 角计算四元数 z/w 即可。
  goal.target_pose.pose.orientation.z = sin(yaw * 0.5);
  goal.target_pose.pose.orientation.w = cos(yaw * 0.5);

  // 发送目标并等待结果，成功返回 0，失败返回 2，便于脚本判断。
  ac.sendGoal(goal);
  bool ok = ac.waitForResult(ros::Duration(120.0));
  if (ok && ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Goal reached.");
    return 0;
  }
  ROS_WARN("Goal failed: %s", ac.getState().toString().c_str());
  return 2;
}
