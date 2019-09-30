#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose2D.h>
#include <random>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>

double pow2(double x) { return x * x; }

void checkGlobalMessage(const std_msgs::String msg) {}

constexpr double MIN_SCAN_LENGTH = 0.25, MAX_SCAN_LENGTH = 7.695, // m
    SCAN_START_ANGLE = -87.0 / 180 * M_PI,
                 SCAN_FINISH_ANGLE = 87.0 / 180 * M_PI; // rad

bool can_detection = false;
sensor_msgs::PointCloud scan_data; // 相対位置
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
      dummy_point.x = msgs.ranges[i] * 1000 *
                      cos(i * msgs.angle_increment + msgs.angle_min);
      dummy_point.y = msgs.ranges[i] * 1000 *
                      sin(i * msgs.angle_increment + msgs.angle_min);
      scan_data.points.push_back(dummy_point);
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
  std::uniform_int_distribution<> point_id(0, scan_data.points.size() - 1);
  geometry_msgs::Point32 sample[3];
  for (int i = 0; i < 3; ++i) {
    sample[i] = scan_data.points.at(point_id(mt));
    ROS_INFO_STREAM(sample[i].x << ", " << sample[i].y);
  }

  LineParam bisector[2];
  int param_id = 0;
  for (int i = 0; i < 3 && param_id < 2; ++i) {
    bisector[param_id].a = -(sample[(i + 1) % 3].x - sample[i].x) /
                           (sample[(i + 1) % 3].y - sample[i].y);
    if (!std::isinf(bisector[param_id].a)) {
      bisector[param_id].b =
          (sample[(i + 1) % 3].y + sample[i].y) / 2 -
          bisector[param_id].a * (sample[(i + 1) % 3].x + sample[i].x) / 2;
    } else {
      continue;
    }
    ROS_INFO_STREAM("bisector: " << param_id << ", " << i << ", "
                                 << bisector[param_id].a << ", "
                                 << bisector[param_id].b);
    ++param_id;
  }

  ModelParam result;
  result.x = (bisector[1].b - bisector[0].b) / (bisector[0].a - bisector[1].a);
  result.y = bisector[0].a * result.x + bisector[0].b;
  ROS_INFO_STREAM(result.x << ", " << result.y);
  return result;
}

sensor_msgs::PointCloud match_scan;
constexpr int MODEL_RADIUS = 720, MAX_ERROR_MODEL = 10, MIN_FIT_POINT = 100,
              MAX_LOOP_COUNT = 200;
ModelParam checkModel() {
  int fit_point_count = 0;
  for (int i = 0; i < MAX_LOOP_COUNT; ++i) {
    fit_point_count = 0;
    match_scan.points.resize(0);
    ModelParam model = makeModel();
    for (point : scan_data.points) {
      if (abs(pow2((point.x - model.x)) + pow2((point.y - model.y)) -
              MODEL_RADIUS) < MAX_ERROR_MODEL) {
        match_scan.points.push_back(point);
        ++fit_point_count;
      }
    }
    if (fit_point_count > MIN_FIT_POINT) {
      return model;
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "lidar_detection_circle");
  ros::NodeHandle n;
  ros::Subscriber scan_sub = n.subscribe("scan", 1, getLidarScan);
  ros::Publisher raw_scan_pub =
      n.advertise<sensor_msgs::PointCloud>("lidar/raw_scan", 1);
  ros::Publisher match_scan_pub =
      n.advertise<sensor_msgs::PointCloud>("lidar/match_scan", 1);
  ros::Publisher robot_pose_pub =
      n.advertise<geometry_msgs::Pose2D>("lidar/robot_pose", 1);
  geometry_msgs::Pose2D robot_pose;
  geometry_msgs::Point32 LIDAR_POSITION; // mm
  LIDAR_POSITION.x = 6250;
  LIDAR_POSITION.y = 5000;
  LIDAR_POSITION.z = 0;

  constexpr int FREQ = 20;
  ros::Rate loop_rate(FREQ);
  while (ros::ok()) {
    ros::spinOnce();

    if (can_detection) {
      ModelParam robot_model = checkModel();
      /* robot_pose.x = LIDAR_POSITION.x - robot_model.y; */
      /* robot_pose.y = LIDAR_POSITION.y + robot_model.x; */
      robot_pose.x = robot_model.x;
      robot_pose.y = robot_model.y;
      ROS_INFO_STREAM(robot_pose.x << ", " << robot_pose.y);
      robot_pose_pub.publish(robot_pose);

      raw_scan_pub.publish(scan_data);
      match_scan_pub.publish(match_scan);
      can_detection = false;
    }
    makeModel();

    loop_rate.sleep();
  }
}
