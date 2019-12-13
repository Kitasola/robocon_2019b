#include <geometry_msgs/Pose2D.h>
#include <random>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>

void checkGlobalMessage(const std_msgs::String msg) {}

constexpr double MIN_SCAN_LENGTH = 0.25, MAX_SCAN_LENGTH = 7.695, // m
    SCAN_START_ANGLE = -87.0 / 180 * M_PI,
                 SCAN_FINISH_ANGLE = 87.0 / 180 * M_PI; // rad
struct Point {
  int x; // mm
  int y; // mm
};
constexpr Point LIDAR_POSITION = {6250, 5000}; // mm

std::vector<Point> scan_data; // 相対位置
scan_data_id_max = 0;
void getLidarScan(const sensor_msgs::LaserScan msgs) {
  int scan_start_id =
      (SCAN_START_ANGLE - msgs.angle_min) / msgs.angle_increment;
  int scan_finish_id =
      (SCAN_FINISH_ANGLE - msgs.angle_min) / msgs.angle_increment;
  scan_data.resize(0);
  for (int i = scan_start_id; i < scan_finish_id + 1; ++i) {
    if (msgs.ranges[i] >= MIN_SCAN_LENGTH &&
        msgs.ranges[i] <= MAX_SCAN_LENGTH) {
      Point dummy_point;
      dummy_point.x =
          msgs.ranges[i] * cos(i * msgs.angle_increment + msgs.angle_min);
      dummy_point.y =
          msgs.ranges[i] * sin(i * msgs.angle_increment + msgs.angle_min);
      scan_data.push_buck(dummy_point);
    }
  }
}

struct ModelParam { // line
  double a;
  double b;
};

std::random_device rnd;
std::mt19937 mt();
ModelParam makeModel() {
  std::uniform_int_distribution<> point_id(0, scan_data.size() - 1);
  Point sample[2] = {scan_data.at(point_id(mt)), scan_data.at(point_id(mt))};
  ModelParam result;
  result.a = (double)(sample[1].y - sample[0].y) / (sample[1].x - sample[0]);
  result.b = sample[1].y - result.a * sample[1].x;
}

constexpr int MAX_ERROR_MODEL = 5;
void checkModel(ModelParam model) {}

int main(int argc, char **argv) {
  ros::init(argc, argv, "lidar_detection_line");
  ros::NodeHandle n;
  ros::Subscriber scan_sub = n.subscribe("scan", 1, getLidarScan);
  ros::Publisher robot_pose_pub =
      n.advertise<geometry_msgs::Pose2D>("lidar/robot_pose", 1);
  geometry_msgs::Pose2D robot_pose;

  mt.seed(rnd());

  constexpr int FREQ = 20;
  ros::Rate loop_rate(FREQ);
  while (ros::ok()) {
    ros::spinOnce();

    robot_pose_pub.publish(robot_pose);

    loop_rate.sleep();
  }
}
