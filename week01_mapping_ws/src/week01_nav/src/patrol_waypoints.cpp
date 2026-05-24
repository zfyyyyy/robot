#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <vector>
#include <cmath>

// 单个巡航点：x、y 是 map 坐标系下的位置，yaw 是机器人朝向，单位为弧度。
struct Waypoint { double x, y, yaw; };

int main(int argc, char** argv) {
  // 初始化 ROS 节点，节点名为 patrol_waypoints。
  ros::init(argc, argv, "patrol_waypoints");

  // move_base 使用 action 通信；这里创建客户端，等待导航服务器接收目标点。
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
  ROS_INFO("Waiting for move_base...");
  ac.waitForServer();
  ROS_INFO("Connected to move_base.");

  // 预设巡航路线。需要根据实际地图修改这些点，确保它们落在可通行区域。
  std::vector<Waypoint> wps = {
    {0.5, 0.0, 0.0},
    {1.5, 0.0, 0.0},
    {1.5, 1.0, 1.57},
    {0.5, 1.0, 3.14}
  };

  for (size_t i = 0; i < wps.size() && ros::ok(); ++i) {
    move_base_msgs::MoveBaseGoal goal;

    // 导航目标必须发布在 map 坐标系下，move_base 会结合 AMCL/TF 计算路径。
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = wps[i].x;
    goal.target_pose.pose.position.y = wps[i].y;

    // 这里只需要平面导航的 yaw 角，将 yaw 转成四元数的 z/w 分量。
    goal.target_pose.pose.orientation.z = sin(wps[i].yaw * 0.5);
    goal.target_pose.pose.orientation.w = cos(wps[i].yaw * 0.5);

    ROS_INFO("Sending waypoint %zu: (%.2f, %.2f, %.2f)", i+1, wps[i].x, wps[i].y, wps[i].yaw);
    ac.sendGoal(goal);

    // 每个目标最多等待 120 秒；失败后继续尝试下一个点，方便观察整条路线表现。
    bool ok = ac.waitForResult(ros::Duration(120.0));
    if (!ok || ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_WARN("Waypoint %zu failed: %s", i+1, ac.getState().toString().c_str());
    } else {
      ROS_INFO("Waypoint %zu reached.", i+1);
    }
  }
  ROS_INFO("Patrol done.");
  return 0;
}
