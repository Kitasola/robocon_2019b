#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <queue>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sstream>
#include <std_msgs/String.h>
#include <string>

constexpr double MIN_SCAN_LENGTH = 0.25, MAX_SCAN_LENGTH = 7.695, // m
    SCAN_START_ANGLE = -87.0 / 180 * M_PI,
                 SCAN_FINISH_ANGLE = 87.0 / 180 * M_PI; // rad
sensor_msgs::PointCloud scan_data;                      // 相対位置
bool can_detection = false;
void getLidarScan(const sensor_msgs::LaserScan msgs) {
  int scan_start_id =
      (SCAN_START_ANGLE - msgs.angle_min) / msgs.angle_increment;
  int scan_finish_id =
      (SCAN_FINISH_ANGLE - msgs.angle_min) / msgs.angle_increment;
  scan_data.points.resize(0);
  for (int i = scan_start_id; i < scan_finish_id + 1; ++i) {
    if (msgs.ranges[i] >= MIN_SCAN_LENGTH &&
        msgs.ranges[i] <= MAX_SCAN_LENGTH) {
      geometry_msgs::Point32 dummy_point;
      dummy_point.x =
          msgs.ranges[i] * cos(i * msgs.angle_increment + msgs.angle_min);
      dummy_point.y =
          msgs.ranges[i] * sin(i * msgs.angle_increment + msgs.angle_min);
      scan_data.points.push_back(dummy_point);
    }
  }
  scan_data.header = msgs.header;
  can_detection = true;
}

std::string getDate() {
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);

  std::stringstream ss;
  ss << std::put_time(std::localtime(&in_time_t), "%Y_%m_%d_%H_%M_%S");
  return ss.str();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_logger");
  ros::NodeHandle n;
  ros::Subscriber scan_sub = n.subscribe("scan", 10, getLidarScan);

  double start = ros::Time::now().toSec();

  std::ofstream log_file;
  std::string log_dir = "/home/kusoelmo/arrc/robocon_2019b/ar/log/lrf/";
  std::string date = getDate();
  log_file.open(log_dir + "scan" + date + ".csv", std::ios::out);
  if (log_file.fail()) {
    ROS_ERROR_STREAM("File Open Failed.");
    std::exit(1);
  }
  ROS_INFO_STREAM("File Open Succeed.");

  while (ros::ok()) {
    ros::spinOnce();

    if (can_detection) {
      double time = ros::Time::now().toSec() - start;
      for (point : scan_data.points) {
        log_file << point.x << ", " << point.y << ", ";
      }
      log_file << time << std::endl;
      can_detection = false;
    }
  }
}
