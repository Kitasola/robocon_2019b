#include <cmath>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <motor_serial/motor_serial.h>
#include <pigpiod.hpp>
#include <ros/ros.h>
#include <sstream>
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
    if (map_.size() > 0) {
      now = map_.at(map_id_);
    }
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
// スイッチ基板のピンアサイン
enum Switch {
  START = 18,
  RESET = 10,
  CALIBRATION = 11,
  SHEET = 12,
  TOWEL_0 = 16,
  TOWEL_1 = 17,
  TOWEL_2 = 18,
  LRF = 19,
  ODOM = 20
};
Switch ALL_SWITCH[] = {START,   RESET,   CALIBRATION, SHEET, TOWEL_0,
                       TOWEL_1, TOWEL_2, LRF,         ODOM};

ros::ServiceClient motor_speed;
int send(int id, int cmd, int data) {
  motor_serial::motor_serial srv;
  srv.request.id = id;
  srv.request.cmd = cmd;
  srv.request.data = data;
  motor_speed.call(srv);
  return srv.response.data;
}

bool can_starts_game = false;
void checkGlobalMessage(const std_msgs::String msg) { can_starts_game = true; }

int main(int argc, char **argv) {
  ros::init(argc, argv, "motion_planner");
  ros::NodeHandle n;
  Ptp planner(&n, "wheel");
  ros::Subscriber global_message_sub =
      n.subscribe("global_message", 1, checkGlobalMessage);
  ros::Publisher global_message_pub =
      n.advertise<std_msgs::String>("global_message", 1);
  std_msgs::String global_message;
  motor_speed = n.serviceClient<motor_serial::motor_serial>("motor_speed");

  // コート情報の取得
  std::string coat_color;
  n.getParam("/coat", coat_color);
  int coat;
  if (coat_color == "blue") {
    coat = 1;
    ROS_WARN_STREAM("Coat is Blue");
  } else if (coat_color == "red") {
    coat = -1;
    ROS_WARN_STREAM("Coat is Red");
  } else {
    coat = 1;
    ROS_WARN_STREAM("Coat is ???");
  }

  // スタート位置の取得
  int start_x, start_y;
  n.getParam("/ar/start_x", start_x);
  n.getParam("/ar/start_y", start_y);

  // パラメータ
  // 2段目昇降機構
  constexpr int TWO_STAGE_ID = 2, TWO_STAGE_HUNGER = 75, TWO_STAGE_SHEET = 75,
                TWO_STAGE_READY = 0, TWO_STAGE_ERROR_MAX = 1; // cm
  constexpr double TWO_STAGE_TIME = 0.06;
  // ハンガー
  constexpr int HUNGER_ID = 1, HUNGER_SPEED = 200;
  constexpr double HUNGER_WAIT_TIME = HUNGER_SPEED * 0.01;
  // 3段目昇降機構
  constexpr int THREE_STAGE_ID = 1, THREE_STAGE_SHEET = 74;
  constexpr double THREE_STAGE_TIME = 0.06;
  // シーツ
  constexpr int SHEET_ID = 3, SHEET_STAGE = 74;
  constexpr double SHEET_STAGE_TIME = 0.06;

  // 座標追加
  // add(int x, int y, int yaw, int action_type = 0, int action_value = 0,
  // int velocity_x = 0, int velocity_y = 0)
  // action_type
  // 0: 通過, 1: 2段目昇降, 2: ハンガー, 3: バスタオル, 4: 3段目昇降, 5:
  // シーツ, 10: 一定時間待機, 11: スタートスイッチ
  constexpr int NUM_MAP = 2;
  // map_type
  // 0: ハンガー, 1: シーツ
  int map_type = 0;
  GoalManager goal_map[NUM_MAP] = {GoalManager(coat), GoalManager(coat)};
  goal_map[0].add(start_x, start_y, 0, 11); // Move: スタートゾーン
  goal_map[0].add(5400, 5500, 0);
  goal_map[0].add(3650, 5500, 0, 1,
                  TWO_STAGE_HUNGER); // Move: 小ポール横 -> Start: 昇降
  goal_map[0].add(
      3650, 5000, 0, 10,
      TWO_STAGE_HUNGER *
          TWO_STAGE_TIME); // Move: 小ポール横 -> Wait: 昇降完了タイマー
  /* goal_map[0].add(3650, 5500, 0); // Move: ハンガー前 */
  goal_map[0].add(
      2850, 5000, 0, 2,
      HUNGER_WAIT_TIME); // Move: ハンガー手前 -> Wait:ハンガー完了タイマー
  goal_map[0].add(2050, 5000, 0, 2,
                  HUNGER_WAIT_TIME); // Move: 次ハンガー手前 -> Wait: ハンガー
  goal_map[0].add(2050, 5500, 0, 2,
                  HUNGER_WAIT_TIME); // Move: 次ハンガー手前 -> Wait: ハンガー
  goal_map[0].add(5400, 5500, 0, 1,
                  TWO_STAGE_READY); // Move: ハンガー前 -> Start: 昇降
  /* goal_map[0].add(5400, 5500, 0);   // Move: 小ポール横 */
  goal_map[0].add(start_x, start_y,
                  0); // Move: スタートゾーン -> Wait: スタートスイッチ
  goal_map[0].restart();

  goal_map[1].add(start_x, start_y,
                  0); // Move: スタートゾーン -> Wait: スタートスイッチ
  goal_map[1].add(5400, 9250, 0); // Move: 大ポール横
  goal_map[1].add(5400, 9000, 0); // Move: 大ポール中央手前
  goal_map[1].add(5400, 9250, 0); // Move: 大ポール横
  goal_map[1].add(start_x, start_y,
                  0); // Move: 大ポール横
  goal_map[1].restart();

  bool changed_phase = true;
  double start;

  // スイッチ基板
  for (Switch pin : ALL_SWITCH) {
    Pi::gpio().set(pin, IN, PULL_DOWN);
  }

  while (ros::ok()) {
    if (Pi::gpio().read(START) == 1) {
      planner.sendNextGoal(goal_map[map_type].getPtp());
      goal_map[map_type].next();
      break;
    }
  }
  global_message.data = "Game Start";
  global_message_pub.publish(global_message);

  constexpr double FREQ = 1;
  ros::Rate loop_rate(FREQ);

  while (ros::ok()) {
    ros::spinOnce();
    /* if (Pi::gpio().read(RESET)) { */
    /*   global_message.data = "Robo_Pose Reset Both"; */
    /*   global_message_pub.publish(global_message); */
    /* } */
    /* if (Pi::gpio().read(ODOM)) { */
    /*   global_message.data = "Robo_Pose ON Odom"; */
    /*   global_message_pub.publish(global_message); */
    /* } else if (Pi::gpio().read(ODOM)) { */
    /*   global_message.data = "Robo_Pose ON LRF"; */
    /*   global_message_pub.publish(global_message); */
    /* } else { */
    /*   global_message.data = "Robo_Pose ON Double"; */
    /*   global_message_pub.publish(global_message); */
    /* } */

    double now = ros::Time::now().toSec();

    bool can_send_next_goal = false;
    if (planner.is_reach_goal) {
      switch (goal_map[map_type].now.action_type) {
      case 0: {
        can_send_next_goal = true;
        changed_phase = true;
        break;
      }
      case 1: {
        start = now;
        // 伸縮
        send(TWO_STAGE_ID, 30, goal_map[map_type].now.action_value);
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
        if (now - start > goal_map[map_type].now.action_value &&
            now - start <= goal_map[map_type].now.action_value * 2) {
          // 縮める
          send(HUNGER_ID, 20, -HUNGER_SPEED);
          // 縮むまでタイマー待機
        } else if (now - start > goal_map[map_type].now.action_value * 2) {
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
        if (now - start > goal_map[map_type].now.action_value) {
          can_send_next_goal = true;
          changed_phase = true;
          break;
        }
      }
      case 11: {
        if (Pi::gpio().read(START) == 1) {
          if (Pi::gpio().read(SHEET)) {
            map_type = 1;
          } else {
            map_type = 0;
          }
          map_type = 0;
          goal_map[map_type].restart();
          can_send_next_goal = true;
          changed_phase = true;
        }
        break;
      }
      }

      if (can_send_next_goal) {
        // ログ取り用
        std::stringstream ss;
        ss << "Log, Done, " << goal_map[map_type].now.x << ", "
           << goal_map[map_type].now.y << ", "
           << goal_map[map_type].now.action_type << ", "
           << goal_map[map_type].now.action_value;
        global_message.data = ss.str();
        global_message_pub.publish(global_message);

        planner.sendNextGoal(goal_map[map_type].getPtp());
        goal_map[map_type].next();
      }

      loop_rate.sleep();
    }
  }
}
