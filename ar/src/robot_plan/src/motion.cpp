#include <cmath>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <motor_serial/motor_serial.h>
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
  int action_type;
  int action_value; // 必要なら使う e.g. 高さ
  int velocity_x;
  int velocity_y;
};

class GoalManager {
public:
  GoalManager(int coat) {
    coat_reverse_ = coat;
    restart();
  }
  void add(GoalInfo goal) {
    goal.x = coat_reverse_ * goal.x;
    goal.yaw = coat_reverse_ * goal.yaw;
    map_.push_back(goal);
  }
  void add(int x, int y, int yaw, int action_type = 0, int action_value = 0,
           int velocity_x = 0, int velocity_y = 0) {
    GoalInfo dummy_goal;
    dummy_goal.x = coat_reverse_ * x;
    dummy_goal.y = y;
    dummy_goal.yaw = coat_reverse_ * yaw;
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
  int coat_reverse_;
};

using namespace ros;
using Pi = Pigpiod;

ros::ServiceClient motor_speed;
int send(int id, int cmd, int data) {
  motor_serial::motor_serial srv;
  srv.request.id = id;
  srv.request.cmd = cmd;
  srv.request.data = data;
  motor_speed.call(srv);
  return srv.response.data;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "motion_planner");
  ros::NodeHandle n;
  Ptp planner(&n, "wheel");
  ros::Publisher global_message_pub =
      n.advertise<std_msgs::String>("global_message", 1);
  std_msgs::String global_message;
  motor_speed = n.serviceClient<motor_serial::motor_serial>("motor_speed");

  constexpr double FREQ = 1, SWITCH_FREQ = 100;
  ros::Rate loop_rate(FREQ);
  ros::Rate switch_rate(SWITCH_FREQ);

  // パラメータ
  // 2段目昇降機構
  constexpr int TWO_STAGE_ID = 2, TWO_STAGE_HUNGER = 74, TWO_STAGE_TOWEL = 100,
                TWO_STAGE_READY = 0, TWO_STAGE_ERROR_MAX = 10; // cm
  constexpr double TWO_STAGE_TIME = 0.06;
  // ハンガー
  constexpr int HUNGER_ID = 1, HUNGER_SPEED = 200;
  constexpr double HUNGER_WAIT_TIME = 1.0;

  // 座標追加
  // add(int x, int y, int yaw, int action_type = 0, int action_value = 0,
  // int velocity_x = 0, int velocity_y = 0)
  // 0: 通過, 1: 2段目昇降, 2: ハンガー, 3: バスタオル, 4: 3段目昇降, 5: シーツ,
  // 10: 一定時間待機
  std::string coat_color;
  n.getParam("/coat", coat_color);
  int coat;
  if (coat_color == "blue") {
    coat = 1;
  } else if (coat_color == "red") {
    coat = -1;
  } else {
    coat = 1;
    ROS_WARN_STREAM("coat???");
  }

  GoalManager goal_map(coat);
  int start_x, start_y;
  n.getParam("/ar/start_x", start_x);
  n.getParam("/ar/start_y", start_y);
  goal_map.add(start_x, start_y, 0); // Move: スタートゾーン
  goal_map.add(5400, 5500, 0, 1,
               TWO_STAGE_HUNGER); // Move: 小ポール横 -> Start: 昇降
  goal_map.add(
      3650, 5500, 0, 10,
      TWO_STAGE_HUNGER *
          TWO_STAGE_TIME); // Move: 小ポール横 -> Wait: 昇降完了タイマー
  /* goal_map.add(-3650, 5500, 0); // Move: ハンガー前 */
  goal_map.add(
      3650, 5000, 0, 2,
      HUNGER_WAIT_TIME); // Move: ハンガー手前 -> Wait:ハンガー完了タイマー
  goal_map.add(2850, 5000, 0, 2,
               HUNGER_WAIT_TIME); // Move: 次ハンガー手前 -> Wait: ハンガー
  goal_map.add(2050, 5000, 0, 2,
               HUNGER_WAIT_TIME); // Move: 次ハンガー手前 -> Wait: ハンガー
  goal_map.add(2050, 5500, 0, 1,
               TWO_STAGE_READY); // Move: ハンガー前 -> Start: 昇降
  goal_map.add(5400, 5500, 0);   // Move: 小ポール横
  goal_map.add(5400, 5500, 0);   // Move: スタートゾーン

  bool changed_phase = true;
  double start;

  // RasPi
  constexpr int START_PIN = 18;
  Pi::gpio().set(18, IN, PULL_DOWN);

  while (ros::ok()) {
    if (Pi::gpio().read(START_PIN)) {
      planner.sendNextGoal(goal_map.getPtp());
      break;
    }
    switch_rate.sleep();
  }
  global_message.data = "Game Start";
  global_message_pub.publish(global_message);

  while (ros::ok()) {
    ros::spinOnce();
    double now = ros::Time::now().toSec();

    bool can_send_next_goal = false;
    if (planner.is_reach_goal) {
      switch (goal_map.now.action_type) {
      case 0: {
        can_send_next_goal = true;
        changed_phase = true;
        break;
      }
      case 1: {
        start = now;
        // 伸縮
        send(TWO_STAGE_ID, 30, goal_map.now.action_value);
        can_send_next_goal = true;
        changed_phase = true;
        break;
      }
      case 2: {
        if (changed_phase) {
          // 伸ばす(取り付け)
          send(HUNGER_ID, 20, HUNGER_SPEED);
          start = now;
          changed_phase = false;
        }
        // 取り付くまでタイマー待機
        if (now - start > goal_map.now.action_value &&
            now - start <= goal_map.now.action_value * 2) {
          // 縮める
          send(HUNGER_ID, 20, -HUNGER_SPEED);
          // 縮むまでタイマー待機
        } else if (now - start > goal_map.now.action_value * 2) {
          can_send_next_goal = true;
          changed_phase = true;
        }
        break;
      }
      case 3: {
        can_send_next_goal = true;
        changed_phase = true;
        break;
      }
      case 10: {
        if (now - start > goal_map.now.action_value) {
          can_send_next_goal = true;
          changed_phase = true;
          break;
        }
      }
      }

      if (can_send_next_goal) {
        // ログ取り用
        global_message.data = "Log, Done, %d, %d, %d, %d", goal_map.now.x,
        goal_map.now.y, goal_map.now.action_type, goal_map.now.action_value;
        global_message_pub.publish(global_message);

        goal_map.next();
        planner.sendNextGoal(goal_map.getPtp());
      }

      loop_rate.sleep();
    }
  }
}
