#include <chrono>
#include <ctime>
#include <fstream>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
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
  double v_x;
  double v_y;
  double v_theta;
};

bool starts_game = false;
bool should_reset_point = false;
std::queue<std::string> log_global_message;
void checkGlobalMessage(const std_msgs::String msg) {
  std::string mode = msg.data;
  if (mode == "Game Start") {
    starts_game = true;
    should_reset_point = true;
  } else if (mode == "Robot Pose Reset") {
    should_reset_point = true;
  }
  log_global_message.push(mode);
}

geometry_msgs::Pose2D robot_pose;
void getPose(const geometry_msgs::Pose2D msgs) { robot_pose = msgs; }

geometry_msgs::Twist robot_velocity;
void getVelocity(const geometry_msgs::Twist msgs) { robot_velocity = msgs; }

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
  ros::Subscriber robot_pose_sub = n.subscribe("robot_pose", 100, getPose);
  ros::Subscriber wheel_velocity_sub =
      n.subscribe("wheel/velocity", 100, getVelocity);
  ros::Subscriber global_sub =
      n.subscribe("global_message", 1, checkGlobalMessage);

  constexpr int FREQ = 100;
  ros::Rate loop_rate(FREQ);
  double start = ros::Time::now().toSec();
  std::queue<LogFormat> log_robot_pose;

  while (ros::ok()) {
    ros::spinOnce();

    if (starts_game) {
      LogFormat data;
      data.time = ros::Time::now().toSec() - start;
      data.x = robot_pose.x;
      data.y = robot_pose.y;
      data.theta = robot_pose.theta;
      data.v_x = robot_velocity.linear.x;
      data.v_y = robot_velocity.linear.y;
      data.v_theta = robot_velocity.angular.z;
      log_robot_pose.push(data);
    }

    loop_rate.sleep();
  }

  if (starts_game) {
    std::ofstream log_file_robot_pose, log_file_motion;
    std::string log_dir = "/home/kusoelmo/arrc/robocon_2019b/ar/log/latest/";
    std::string date = getDate();
    log_file_robot_pose.open(log_dir + "robot_pose" + date + ".csv",
                             std::ios::out);
    log_file_motion.open(log_dir + "motion" + date + ".csv", std::ios::out);
    if (log_file_robot_pose.fail()) {
      ROS_ERROR_STREAM("File(robot_pose) Open Failed.");
      std::exit(1);
    }
    if (log_file_motion.fail()) {
      ROS_ERROR_STREAM("File(motion) Open Failed.");
      std::exit(1);
    }
    ROS_INFO_STREAM("File Open Succeed.");
    while (!log_robot_pose.empty()) {
      LogFormat data = log_robot_pose.front();
      log_file_robot_pose << data.time << ", " << data.x << ", " << data.y
                          << ", " << data.theta << ", " << data.v_x << ", "
                          << data.v_y << ", " << data.v_theta << std::endl;
      log_robot_pose.pop();
    }
    while (!log_global_message.empty()) {
      if (log_global_message.front().find("Log")) {
        log_file_motion << log_global_message.front() << std::endl;
      }
      log_global_message.pop();
    }
  }
}
