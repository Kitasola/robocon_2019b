#include <geometry_msgs/Pose2D.h>
#include <ros/ros.h>
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
  geometry_msgs::Pose2D robot_pose;

  constexpr int FREQ = 100;
  ros::Rate loop_rate(FREQ);

  geometry_msgs::Pose2D offset_robot_pose;
  int start_x, start_y;
  n.getParam("/ar/start_x", start_x);
  n.getParam("/ar/start_y", start_y);

  offset_robot_pose.x = 0;
  offset_robot_pose.y = 0;
  offset_robot_pose.theta = 0;

  geometry_msgs::Pose2D robot_relative_pose;
  while (ros::ok()) {
    ros::spinOnce();

    robot_relative_pose = wheel_robot_pose;
    if (should_reset_point) {
      offset_robot_pose = robot_relative_pose;
      should_reset_point = false;
    }

    robot_relative_pose.x -= offset_robot_pose.x;
    robot_relative_pose.y -= offset_robot_pose.y;
    robot_relative_pose.theta -= offset_robot_pose.theta;
    if (robot_relative_pose.theta > M_PI) {
      robot_relative_pose.theta -= 2 * M_PI;
    } else if (robot_relative_pose.theta < -M_PI) {
      robot_relative_pose.theta += 2 * M_PI;
    }

    // 相対位置を絶対位置に変換する
    robot_pose = robot_relative_pose;
    robot_pose.x += start_x;
    robot_pose.y += start_y;
    robot_pose_pub.publish(robot_pose);

    loop_rate.sleep();
  }
}
