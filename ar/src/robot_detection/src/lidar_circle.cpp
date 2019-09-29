#include <geometry_msgs/Pose2D.h>
#include <random>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>

double pow2(double x) { return x * x; }

void checkGlobalMessage(const std_msgs::String msg) {}

constexpr double MIN_SCAN_LENGTH = 0.25, MAX_SCAN_LENGTH = 7.695, // m
    SCAN_START_ANGLE = -87.0 / 180 * M_PI,
                 SCAN_FINISH_ANGLE = 87.0 / 180 * M_PI; // rad
struct Point {
  int x; // mm
  int y; // mm
};

bool can_detection = false;
std::vector<Point> scan_data; // 相対位置
void getLidarScan(const sensor_msgs::LaserScan msgs) {
  int scan_start_id =
      (SCAN_START_ANGLE - msgs.angle_min) / msgs.angle_increment;
  int scan_finish_id =
      (SCAN_FINISH_ANGLE - msgs.angle_min) / msgs.angle_increment;
  scan_data.resize(0);
  ROS_INFO_STREAM("OK0");
  for (int i = scan_start_id; i < scan_finish_id + 1; ++i) {
    ROS_INFO_STREAM("OK1:" << i);
    if (msgs.ranges[i] >= MIN_SCAN_LENGTH &&
        msgs.ranges[i] <= MAX_SCAN_LENGTH) {
      Point dummy_point;
      dummy_point.x = msgs.ranges[i] * 1000 *
                      cos(i * msgs.angle_increment + msgs.angle_min);
      dummy_point.y = msgs.ranges[i] * 1000 *
                      sin(i * msgs.angle_increment + msgs.angle_min);
      scan_data.push_back(dummy_point);
    }
  }
  can_detection = true;
}

struct ModelParam { // Circle
  double x;
  double y;
};

struct LineParam {
  double a;
  double b;
};

std::random_device rnd;
std::mt19937 mt(rnd());
ModelParam makeModel() {
  std::uniform_int_distribution<> point_id(0, scan_data.size() - 1);
  Point sample[3];
  for (int i = 0; i < 3; ++i) {
    sample[i] = scan_data.at(point_id(mt));
  }
  ROS_INFO_STREAM("OK2");

  LineParam bisector[2];
  for (int i = 0; i < 2; ++i) {
    bisector[i].a = (double)(sample[i + 1].x - sample[i].x) /
                    (sample[i + 1].y - sample[i].y);
    bisector[i].b = (double)(sample[i + 1].y + sample[i].y) / 2 -
                    bisector[i].a * (double)(sample[i + 1].x + sample[i].x) / 2;
  }
  ROS_INFO_STREAM("OK3");

  ModelParam result;
  result.x = (bisector[1].b - bisector[0].b) / (bisector[0].a - bisector[1].a);
  result.y = bisector[0].a * result.x + bisector[0].b;
  return result;
}

constexpr int MODEL_RADIUS = 720, MAX_ERROR_MODEL = 5, MIN_FIT_POINT = 100,
              MAX_LOOP_COUNT = 10;
ModelParam checkModel() {
  int fit_point_count = 0;
  int loop_count = 0;
  while (fit_point_count < MIN_FIT_POINT || loop_count < MAX_LOOP_COUNT) {
    fit_point_count = 0;
    ModelParam model = makeModel();
    for (point : scan_data) {
      if (abs(pow2((point.x - model.x)) + pow2((point.y - model.y)) -
              MODEL_RADIUS) < MAX_ERROR_MODEL) {
        ++fit_point_count;
      }
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "lidar_detection_circle");
  ros::NodeHandle n;
  ros::Subscriber scan_sub = n.subscribe("scan", 1, getLidarScan);
  ros::Publisher robot_pose_pub =
      n.advertise<geometry_msgs::Pose2D>("lidar/robot_pose", 1);
  geometry_msgs::Pose2D robot_pose;
  Point LIDAR_POSITION = {6250, 5000}; // mm

  constexpr int FREQ = 20;
  ros::Rate loop_rate(FREQ);
  while (ros::ok()) {
    ros::spinOnce();

    if (can_detection) {
      ModelParam robot_model = checkModel();
      robot_pose.x = LIDAR_POSITION.x - robot_model.y;
      robot_pose.y = LIDAR_POSITION.y + robot_model.x;
      robot_pose_pub.publish(robot_pose);
      can_detection = false;
    }

    loop_rate.sleep();
  }
}
