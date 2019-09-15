#include <cmath>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <pigpiod.hpp>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <vector>

class Ptp {
public:
  Ptp(ros::NodeHandle *n, const std::string usename) {
    goal_point_pub =
        n->advertise<geometry_msgs::Pose2D>(usename + "/goal_point", 1);
    reach_goal_sub =
        n->subscribe(usename + "/reach_goal", 1, &Ptp::checkReachGoal, this);
  }

  bool is_reach_goal;
  void checkReachGoal(const std_msgs::Bool &msg) { is_reach_goal = msg.data; }
  void sendNextGoal(geometry_msgs::Pose2D goal_point) {
    /* ROS_INFO_STREAM("Next Goal Point is " << goal_point.x << ", " */
    /*                                       << goal_point.y << ", " */
    /*                                       << goal_point.theta * 180 / M_PI);
     */
    goal_point_pub.publish(goal_point);
    is_reach_goal = false;
  }

  void sendNextGoal(double x, double y, double theta) {
    geometry_msgs::Pose2D dummy_goal;
    dummy_goal.x = x;
    dummy_goal.y = y;
    dummy_goal.theta = theta;
    sendNextGoal(dummy_goal);
  }

private:
  std::queue<geometry_msgs::Pose2D> goal_point_list;
  ros::Subscriber reach_goal_sub;
  ros::Publisher goal_point_pub;
  geometry_msgs::Pose2D goal_point = {};
};

using namespace ros;
using Pi = Pigpiod;

struct GoalInfo {
  int x;
  int y;
  int velocity_x;
  int velocity_y;
  int yaw; // degree あとでradに変換する
  int action_type; // 0: 通過, 1: 2段目昇降, 2: ハンガー, 3: バスタオル, 4:
                   // 3段目昇降, 5: シーツ
  int action_value // 必要なら使う e.g. 高さ
};

class GoalManager

    int
    main(int argc, char **argv) {
  ros::init(argc, argv, "motion_planner");
  ros::NodeHandle n;
  Ptp planner(&n, "wheel");

  constexpr double FREQ = 300;
  ros::Rate loop_rate(FREQ);

  // 座標追加
  std::vector goal_map<GoalInfo>;
  int goal_map_id = 0;
  goal_map.push_buck();

  /* constexpr int START_PIN = 18; */
  /* Pi::gpio().set(18, IN, PULL_DOWN); */

  /* while (!Pi::gpio().read(START_PIN)) { */
  /*   loop_rate.sleep(); */
  /* }; */

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}
