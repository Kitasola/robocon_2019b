#include <chrono>
#include <ctime>
#include <fstream>
#include <geometry_msgs/Pose2D.h>
#include <iomanip>
#include <queue>
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/String.h>
#include <string>

struct LogFormat {
  double time;
  double x;
  double y;
  double theta;
};

bool starts_game = false;
bool should_reset_point = false;
void checkGlobalMessage(const std_msgs::String msg) {
  std::string mode = msg.data;
  if (mode == "Game Start") {
    starts_game = true;
  } else if (mode == "Robot Pose Reset") {
    should_reset_point = true;
  }
}

// MDDからは起動時からの相対位置が送信されるため絶対位置に変換する必要がある
constexpr double FIRST_X = 5400, FIRST_Y = 1800;
geometry_msgs::Pose2D wheel_robot_pose;
void getPoseWheel(const geometry_msgs::Pose2D msgs) {
  wheel_robot_pose.x = msgs.x * 1.0 + FIRST_X;
  wheel_robot_pose.y = msgs.y * 1.0 + FIRST_Y;
  wheel_robot_pose.theta = msgs.theta * 1.0;
}

std::string getDate() {
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);

  std::stringstream ss;
  ss << std::put_time(std::localtime(&in_time_t), "%Y_%m_%d_%H_%M_%S");
  return ss.str();
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

  constexpr int FREQ = 300;
  ros::Rate loop_rate(FREQ);
  double start = ros::Time::now().toSec();
  std::queue<LogFormat> log_data;

  geometry_msgs::Pose2D offset_robot_pose;
  while (ros::ok()) {
    ros::spinOnce();

    if (should_reset_point) {
      offset_robot_pose = robot_pose;
      should_reset_point = false;
    }
    robot_pose = wheel_robot_pose;

    robot_pose.x = robot_pose.x - offset_robot_pose.x;
    robot_pose.y = robot_pose.y - offset_robot_pose.y;
    robot_pose.theta = robot_pose.theta - offset_robot_pose.theta;
    robot_pose_pub.publish(robot_pose);

    LogFormat data;
    data.time = ros::Time::now().toSec() - start;
    data.x = robot_pose.x;
    data.y = robot_pose.y;
    data.theta = robot_pose.theta;
    log_data.push(data);

    loop_rate.sleep();
  }

  std::ofstream log;
  std::string log_dir = "/home/kusoelmo/arrc/robocon_2019b/ar/log/";
  log.open(log_dir + "robot_pose_" + getDate() + ".csv", std::ios::out);
  if (log.fail()) {
    ROS_ERROR_STREAM("File Open Failed.");
    std::exit(1);
  }
  ROS_INFO_STREAM("File Open Succeed.");
  LogFormat data_prev_prev = log_data.front();
  LogFormat data_prev = log_data.front();
  while (!log_data.empty()) {
    LogFormat data = log_data.front();
    double velocity_x = (data_prev.x - data_prev_prev.x) /
                        (data_prev.time - data_prev_prev.time);
    double velocity_y = (data_prev.y - data_prev_prev.y) /
                        (data_prev.time - data_prev_prev.time);
    log << data.time << ", " << data.x << ", " << data.y << ", " << data.theta
        << ", " << velocity_x << ", " << velocity_y << std::endl;
    data_prev_prev = data_prev;
    data_prev = data;
    log_data.pop();
  }
}
