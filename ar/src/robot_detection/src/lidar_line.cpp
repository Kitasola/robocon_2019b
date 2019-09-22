#include <geometry_msgs/Pose2D.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <string>

void checkGlobalMessage(const std_msgs::String msg) {}

constexpr double MIN_SCAN_LENGTH = 0.5, MAX_SCAN_LENGTH = 10,
                 MIN_SCAN_ID_START =,
                 MAX_SCAN_ID_FINISH = 180 / 180 * M_PI; // m
void getLidarScan(const sensor_msgs::LaserScan msgs) {}

int main(int argc, char **argv) {
  ros::init(argc, argv, "lidar_robot_detection");
  ros::NodeHandle n;
  ros::Subscriber scan_sub = n.subscribe("scan", 1, getLidarScan);
  ros::Publisher robot_pose_pub =
      n.advertise<geometry_msgs::Pose2D>("lidar/robot_pose", 1);
  geometry_msgs::Pose2D robot_pose;

  constexpr int FREQ = 10;
  ros::Rate loop_rate(FREQ);
  while (ros::ok()) {
    ros::spinOnce();

    robot_pose_pub.publish(robot_pose);

    loop_rate.sleep();
  }
}
