#include <geometry_msgs/Pose2D.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <string>

bool should_reset_point = false;
void checkGlobalMessage(const std_msgs::String msg) {
  std::string mode = msg.data;
  if (mode == "Game Start") {
    should_reset_point = true;
  } else if (mode == "Robot Pose Reset") {
    should_reset_point = true;
  }
}

geometry_msgs::Pose2D wheel_robot_pose;
void getPoseWheel(const geometry_msgs::Pose2D msgs) {
  wheel_robot_pose.x = msgs.x * 1.0;
  wheel_robot_pose.y = msgs.y * 1.0;
  wheel_robot_pose.theta = msgs.theta * 1.0;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "dead_reckoning");
  ros::NodeHandle n;
  ros::Subscriber wheel_robot_pose_sub =
      n.subscribe("wheel/robot_pose", 1, getPoseWheel);
  ros::Subscriber global_sub =
      n.subscribe("global_message", 1, checkGlobalMessage);
  /* ros::Subscriber lidar_robot_pose_sub = */
  /*     n.subscribe("lidar/robot_pose", 1, getPoseLidar); */
  ros::Publisher robot_pose_pub =
      n.advertise<geometry_msgs::Pose2D>("robot_pose", 1);
  ros::Publisher reset_robot_pose_pub =
      n.advertise<geometry_msgs::Pose2D>("wheel/reset_robot_pose", 1);
  geometry_msgs::Pose2D robot_pose;

  constexpr int FREQ = 100;
  ros::Rate loop_rate(FREQ);

  // コート情報の取得
  std::string coat_color;
  n.getParam("/coat", coat_color);
  int coat;
  if (coat_color == "blue") {
    coat = 1;
  } else if (coat_color == "red") {
    coat = -1;
  } else {
    coat = 1;
  }

  geometry_msgs::Pose2D offset_robot_pose;
  int start_x, start_y;
  double start_yaw;
  n.getParam("/ar/start_x", start_x);
  n.getParam("/ar/start_y", start_y);
  n.getParam("/ar/start_yaw", start_yaw);
  start_x *= coat;
  start_yaw = coat * start_yaw / 180.0 * M_PI;

  offset_robot_pose.x = 0;
  offset_robot_pose.y = 0;
  offset_robot_pose.theta = 0;

  geometry_msgs::Pose2D robot_relative_pose;
  while (ros::ok()) {

    ros::Time now = ros::Time::now();
    if (should_reset_point) {
      robot_pose.x = start_x;
      robot_pose.y = start_y;
      robot_pose.theta = start_yaw;
      if (robot_pose.theta > M_PI) {
        robot_pose.theta -= 2 * M_PI;
      } else if (robot_pose.theta <= -M_PI) {
        robot_pose.theta += 2 * M_PI;
      }
      should_reset_point = false;
    } else {
      robot_pose = wheel_robot_pose;
      if (robot_pose.theta > M_PI) {
        robot_pose.theta -= 2 * M_PI;
      } else if (robot_pose.theta <= -M_PI) {
        robot_pose.theta += 2 * M_PI;
      }
      robot_pose_pub.publish(robot_pose);
    }

    loop_rate.sleep();
  }
}
