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
  ros::Subscriber reach_goal_sub;
  ros::Publisher goal_point_pub;
  geometry_msgs::Pose2D goal_point = {};
};

using namespace ros;
using Pi = Pigpiod;

struct GoalInfo {
  int x;
  int y;
  int yaw; // degree あとでradに変換する
  int action_type; // 0: 通過, 1: 2段目昇降, 2: ハンガー, 3: バスタオル, 4:
                   // 3段目昇降, 5: シーツ
  int action_value; // 必要なら使う e.g. 高さ
  int velocity_x;
  int velocity_y;
};

class GoalManager {
public:
  void add(GoalInfo goal) { map_.push_back(goal); }
  void add(int x, int y, int yaw, int action_type = 0, int action_value = 0,
           int velocity_x = 0, int velocity_y = 0) {
    GoalInfo dummy_goal;
    dummy_goal.x = x;
    dummy_goal.y = y;
    dummy_goal.yaw = yaw;
    dummy_goal.action_type = action_type;
    dummy_goal.action_value = action_value;
    dummy_goal.velocity_x = velocity_x;
    dummy_goal.velocity_y = velocity_y;
    map_.push_back(dummy_goal);
  }

  void next() {
    if (map_id_ < map_.size()) {
      ++map_id_;
      now = map_.at(map_id_);
    }
  }

  void restart() {
    map_id_ = -1;
    next();
  };

  void reset() { map_.resize(0); }

  geometry_msgs::Pose2D getPtp() {
    geometry_msgs::Pose2D dummy_goal;
    dummy_goal.x = now.x;
    dummy_goal.y = now.y;
    dummy_goal.theta = now.yaw / 180.0 * M_PI;
    return dummy_goal;
  }

  GoalInfo now;

private:
  std::vector<GoalInfo> map_;
  int map_id_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "motion_planner");
  ros::NodeHandle n;
  Ptp planner(&n, "wheel");

  constexpr double FREQ = 300;
  ros::Rate loop_rate(FREQ);

  // 座標追加
  GoalManager goal_map;
  goal_map.add(5400, 5500, 0);
  goal_map.add(3600, 5500, 0);
  goal_map.add(3600, 5250, 0);
  goal_map.add(3600, 5500, 0);
  goal_map.add(5400, 5500, 0);

  constexpr int START_PIN = 18;
  Pi::gpio().set(18, IN, PULL_DOWN);

  while (!Pi::gpio().read(START_PIN)) {
    loop_rate.sleep();
  };
  planner.sendNextGoal(goal_map.getPtp());
  goal_map.next();

  while (ros::ok()) {
    ros::spinOnce();

    bool is_send_next_goal = false;
    if (planner.is_reach_goal) {
      switch (goal_map.now.action_type) {
      case 0:
        is_send_next_goal = true;
        break;
      }
    }

    if (is_send_next_goal) {
      goal_map.next();
      planner.sendNextGoal(goal_map.getPtp());
    }

    loop_rate.sleep();
  }
}
