#include <cmath>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <motor_serial.hpp>
#include <pigpiod.hpp>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
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
    ROS_INFO_STREAM("Next Goal Point is " << goal_point.x << ", "
                                          << goal_point.y << ", "
                                          << goal_point.theta * 180 / M_PI);
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
  GoalManager() {
    add(5400, 1800, 0);
    restart();
  }
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

  void show() {
    for (x : map_) {
      ROS_INFO_STREAM(x.x << ", " << x.y);
    }
  }

  void next() {
    if (map_id_ < map_.size() - 1) {
      ++map_id_;
      now = map_.at(map_id_);
    }
  }

  void restart() {
    map_id_ = 0;
    now = map_.at(map_id_);
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
  int map_id_ = 0;
};

using namespace ros;
using Pi = Pigpiod;

int main(int argc, char **argv) {
  ros::init(argc, argv, "motion_planner");
  ros::NodeHandle n;
  Ptp planner(&n, "wheel");
  ros::Publisher global_message_pub =
      n.advertise<std_msgs::String>("global_message", 1);
  std_msgs::String global_message;

  constexpr double FREQ = 10;
  ros::Rate loop_rate(FREQ);

  // パラメータ
  // 2段目昇降機構
  constexpr int TWO_STAGE_ID = 2, TWO_STAGE_HUNGER = 60, TWO_STAGE_TOWEL = 100,
                TWO_STAGE_READY = 0, TWO_STAGE_ERROR_MAX = 10; // cm

  // 座標追加
  // add(int x, int y, int yaw, int action_type = 0, int action_value = 0, int
  // velocity_x = 0, int velocity_y = 0)
  GoalManager goal_map;
  goal_map.add(5400, 5500, 0);
  goal_map.add(3650, 5500, 0, 1, TWO_STAGE_HUNGER);
  // ハンガー前
  goal_map.add(3650, 5000, 0, 3, TWO_STAGE_HUNGER);
  goal_map.add(3650, 5000, 0, 1, TWO_STAGE_READY);
  goal_map.add(3650, 5500, 0);
  goal_map.add(5400, 5500, 0);
  goal_map.add(5400, 1800, 0);

  // RasPi
  // MotorSerial
  MotorSerial ms;
  constexpr int START_PIN = 18;
  Pi::gpio().set(18, IN, PULL_DOWN);

  while (ros::ok()) {
    if (Pi::gpio().read(START_PIN)) {
      planner.sendNextGoal(goal_map.getPtp());
      goal_map.next();
      break;
    }
    loop_rate.sleep();
  }
  global_message.data = "Game Start";
  global_message_pub.publish(global_message);

  while (ros::ok()) {
    ros::spinOnce();

    bool can_send_next_goal = false;
    if (planner.is_reach_goal) {
      switch (goal_map.now.action_type) {
      case 0: {
        can_send_next_goal = true;
        break;
      }
      case 1: {
        int height = ms.send(TWO_STAGE_ID, 30, goal_map.now.action_value);
        can_send_next_goal = true;
        break;
      }
      case 3: {
        int height = ms.send(TWO_STAGE_ID, 33, 10);
        ROS_INFO_STREAM("height" << height);
        if (height > goal_map.now.action_value - TWO_STAGE_ERROR_MAX &&
            height < goal_map.now.action_value + TWO_STAGE_ERROR_MAX) {
          can_send_next_goal = true;
        }
        break;
      }
      }

      if (can_send_next_goal) {
        planner.sendNextGoal(goal_map.getPtp());
        goal_map.next();
      }

      loop_rate.sleep();
    }
  }
}
