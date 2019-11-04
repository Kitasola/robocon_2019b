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

double abs(double a) { return fabs(a); }

class Ptp {
public:
  Ptp(ros::NodeHandle *n, const std::string usename) {
    goal_point_pub =
        n->advertise<geometry_msgs::Pose2D>(usename + "/goal_point", 1);
    emergency_stop_pub =
        n->advertise<std_msgs::Bool>(usename + "/emergency_stop", 1);
    robot_pose_sub = n->subscribe("robot_pose", 1, &Ptp::getRobotPose, this);
  }

  void getRobotPose(const geometry_msgs::Pose2D &msg) { current_point = msg; }
  /* void getRobotPose(const geometry_msgs::Pose &msgs) { */
  /* current_point.x = msgs.position.x; */
  /* current_point.y = msgs.position.y; */
  /* double x = msgs.orientation.x, w = msgs.orientation.w; */
  /* current_point.theta = atan2(2 * x * w, x * x - w * w); */
  /* } */

  bool should_stop_emergency = false;
  bool checkReachGoal(double error_distance, double error_angle) {
    double diff_yaw = (goal_point.theta - current_point.theta) / M_PI * 180;
    diff_yaw = diff_yaw - (int)diff_yaw / 180 * 360;
    /* ROS_INFO_STREAM("x:" << goal_point.x << ", " << current_point.x << ", "
     */
    /*                      << goal_point.y << ", " << current_point.y << ",
     * yaw: " */
    /*                      << goal_point.theta << ", " << current_point.theta);
     */
    if (hypot(goal_point.x - current_point.x, goal_point.y - current_point.y) <
            error_distance &&
        abs(diff_yaw) < error_angle) {
      return true;
    } else {
      return false;
    }
  }

  void sendEmergencyStatus() {
    std_msgs::Bool msg;
    msg.data = should_stop_emergency;
    emergency_stop_pub.publish(msg);
  }

  void sendNextGoal(geometry_msgs::Pose2D point) {
    goal_point = point;
    ROS_INFO_STREAM("Next Goal Point is " << goal_point.x << ", "
                                          << goal_point.y << ", "
                                          << goal_point.theta * 180 / M_PI);
    goal_point_pub.publish(goal_point);
    should_stop_emergency = false;
    sendEmergencyStatus();
  }

  void sendNextGoal(double x, double y, double theta) {
    geometry_msgs::Pose2D dummy_goal;
    dummy_goal.x = x;
    dummy_goal.y = y;
    dummy_goal.theta = theta;
    sendNextGoal(dummy_goal);
  }

private:
  ros::Publisher goal_point_pub, emergency_stop_pub;
  ros::Subscriber robot_pose_sub;
  geometry_msgs::Pose2D goal_point = {}, current_point = {};
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
  EMERGENCY = 19,
  RESET = 10,
  CALIBRATION = 11,
  HUNGER_1 = 22,
  HUNGER_2 = 23,
  HUNGER_3 = 24
};
Switch ALL_SWITCH[] = {START,    EMERGENCY, RESET,   CALIBRATION,
                       HUNGER_1, HUNGER_2,  HUNGER_3};

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
    ROS_INFO_STREAM("Coat is Blue");
  } else if (coat_color == "red") {
    coat = -1;
    ROS_INFO_STREAM("Coat is Red");
  } else {
    coat = 1;
    ROS_INFO_STREAM("Coat is ???");
  }

  // スタート位置の取得
  int start_x, start_y;
  double start_yaw;
  n.getParam("/ar/start_x", start_x);
  n.getParam("/ar/start_y", start_y);
  n.getParam("/ar/start_yaw", start_yaw);

  // パラメータ
  constexpr double ERROR_DISTANCE_MAX = 50, ERROR_ANGLE_MAX = 1.0;
  // 2段目昇降機構
  constexpr int TWO_STAGE_ID = 2, TWO_STAGE_HUNGER = 75, TWO_STAGE_TOWEL = 77,
                TWO_STAGE_SHEET = 77, TWO_STAGE_READY = 0,
                TWO_STAGE_ERROR_MAX = 1; // cm
  constexpr double TWO_STAGE_TIME = 0.06;
  // ハンガー
  constexpr int HUNGER_POSITION_Y = 4500 - 500 - 100;
  constexpr int HUNGER_ID = 1, HUNGER_SPEED = 200;
  constexpr double HUNGER_WAIT_TIME = HUNGER_SPEED * 0.02;
  // タオル
  constexpr int TOWEL_POSITION_Y = 6000 + 500 + 100;
  constexpr int TOWEL_ID = 1, NUM_TOWEL = 3;
  constexpr int TOWEL_ANGLE[NUM_TOWEL] = {20, 50, -20};
  constexpr double TOWEL_WAIT_TIME = 3;
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
  constexpr int NUM_MAP = 3;
  // map_type
  // 0: ハンガー, 1: シーツ
  int map_type = 1;
  GoalManager goal_map[NUM_MAP] = {GoalManager(coat), GoalManager(coat),
                                   GoalManager(coat)};
  //位置: 後判定
  goal_map[0].add(start_x, start_y, start_yaw, 11); // Move: スタートゾーン
  goal_map[0].add(3650, HUNGER_POSITION_Y, start_yaw);
  goal_map[0].add(3650, HUNGER_POSITION_Y, start_yaw, 1,
                  TWO_STAGE_HUNGER); // Move: 小ポール横 -> Start: 昇降
  goal_map[0].add(
      3650, HUNGER_POSITION_Y, start_yaw, 10,
      TWO_STAGE_HUNGER *
          TWO_STAGE_TIME); // Move: 小ポール横 -> Wait: 昇降完了タイマー
  goal_map[0].add(
      2850, 3850, start_yaw, 2,
      HUNGER_WAIT_TIME); // Move: ハンガー手前 -> Wait:ハンガー完了タイマー
  goal_map[0].add(2050, 3850, start_yaw, 2,
                  HUNGER_WAIT_TIME); // Move: 次ハンガー手前 -> Wait:ハンガー
  goal_map[0].add(2050, HUNGER_POSITION_Y, start_yaw, 2,
                  HUNGER_WAIT_TIME); // Move: 次ハンガー手前 -> Wait: ハンガー
  goal_map[0].add(2050, HUNGER_POSITION_Y, start_yaw, 1,
                  TWO_STAGE_READY); // Move: 次ハンガー手前 -> Wait: ハンガー
  goal_map[0].add(
      start_x + 200, start_y - 200, start_yaw, 10,
      TWO_STAGE_HUNGER *
          TWO_STAGE_TIME); // Move: スタートゾーン -> Wait: スタートスイッチ
  goal_map[0].add(start_x + 200, start_y - 200, start_yaw,
                  11); // Move: スタートゾーン -> Wait: スタートスイッチ
  goal_map[0].restart();

  goal_map[1].add(start_x, start_y, start_yaw, 11); // Move: スタートゾーン
  goal_map[1].add(5400, 7500, start_yaw);
  goal_map[1].add(3650, 7500, start_yaw, 1, TWO_STAGE_TOWEL);
  goal_map[1].add(3650, TOWEL_POSITION_Y, start_yaw, 10,
                  TWO_STAGE_HUNGER); // Move: 小ポール横 -> Start: 昇降
  goal_map[1].add(2850, 7500, start_yaw, 3, TOWEL_ANGLE[0]);
  goal_map[1].add(2850, 7500, start_yaw);
  goal_map[1].add(2050, TOWEL_POSITION_Y, start_yaw, 3, TOWEL_ANGLE[1]);
  goal_map[1].add(2050, 7500, start_yaw);
  goal_map[1].add(2050, 7500, start_yaw, 3, TOWEL_ANGLE[2]);
  goal_map[1].add(2050, 7500, start_yaw, 1,
                  TWO_STAGE_READY); // Move: 次ハンガー手前 -> Wait: ハンガー
  goal_map[1].add(
      5400, 7500, start_yaw, 10,
      TWO_STAGE_HUNGER *
          TWO_STAGE_TIME); // Move: スタートゾーン -> Wait: スタートスイッチ
  goal_map[1].add(start_x + 200, start_y - 200, start_yaw);
  goal_map[1].add(start_x + 200, start_y - 200, start_yaw,
                  11); // Move: スタートゾーン -> Wait: スタートスイッチ
  goal_map[1].restart();

  goal_map[2].add(start_x, start_y, start_yaw, 11); // Move: スタートゾーン
  goal_map[2].add(5400, 7500, start_yaw);
  goal_map[2].add(5400, 7500, start_yaw, 10,
                  TWO_STAGE_TOWEL); // Move: 小ポール横 -> Start: 昇降
  goal_map[2].add(
      5400, 7500, start_yaw, 10,
      TWO_STAGE_TOWEL *
          TWO_STAGE_TIME); // Move: スタートゾーン -> Wait: スタートスイッチ
  goal_map[2].add(5400, 7500, start_yaw, 10,
                  TWO_STAGE_READY); // Move: 小ポール横 -> Start: 昇降
  goal_map[2].add(
      5400, 7500, start_yaw, 10,
      TWO_STAGE_TOWEL *
          TWO_STAGE_TIME); // Move: スタートゾーン -> Wait: スタートスイッチ
  goal_map[2].add(start_x + 200, start_y - 200, start_yaw,
                  11); // Move: スタートゾーン -> Wait: スタートスイッチ
  goal_map[2].restart();

  bool changed_phase = true;
  double start;

  // スイッチ基板
  constexpr double FREQ = 10;
  ros::Rate loop_rate(FREQ);

  for (Switch pin : ALL_SWITCH) {
    Pi::gpio().set(pin, IN, PULL_DOWN);
  }

  while (ros::ok()) {
    loop_rate.sleep();
    if (Pi::gpio().read(START) == 1) {
      planner.sendNextGoal(goal_map[map_type].getPtp());
      goal_map[map_type].next();
      break;
    }
    planner.should_stop_emergency = true;
    planner.sendEmergencyStatus();
  }
  global_message.data = "Game Start";
  global_message_pub.publish(global_message);

  while (ros::ok()) {
    ros::spinOnce();
    if (Pi::gpio().read(EMERGENCY) == 0) {
      send(4, 100, 0);
      send(4, 101, 0);
    } else {
      send(4, 100, 2);
      send(4, 101, 2);
    }
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
    if (planner.checkReachGoal(ERROR_DISTANCE_MAX, ERROR_ANGLE_MAX)) {
      switch (goal_map[map_type].now.action_type) {
      case 0: {
        can_send_next_goal = true;
        changed_phase = true;
        break;
      }
      case 1: {
        planner.should_stop_emergency = true;
        start = now;
        // 伸縮
        send(TWO_STAGE_ID, 30, goal_map[map_type].now.action_value);
        can_send_next_goal = true;
        changed_phase = true;
        break;
      }
      case 2: {
        planner.should_stop_emergency = true;
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
        planner.should_stop_emergency = true;
        if (changed_phase) {
          // 伸ばす(取り付け)
          send(TOWEL_ID, 10, goal_map[map_type].now.action_value);
          start = now;
          changed_phase = false;
        }
        // 取り付くまでタイマー待機
        if (now - start > TOWEL_WAIT_TIME) {
          send(TOWEL_ID, 10, 0);
          can_send_next_goal = true;
          changed_phase = true;
        }
        break;
      }
      case 10: {
        if (now - start > goal_map[map_type].now.action_value) {
          planner.should_stop_emergency = true;
          can_send_next_goal = true;
          changed_phase = true;
          break;
        }
      }
      case 11: {
        if (Pi::gpio().read(START) == 1) {
          if (Pi::gpio().read(HUNGER_1) == 1) {
            map_type = 0;
          } else if (Pi::gpio().read(HUNGER_2) == 1) {
            map_type = 1;
          } else if (Pi::gpio().read(HUNGER_3) == 1) {
            map_type = 2;
          }
          map_type = 1;
          global_message.data = "Robot Pose Reset";
          global_message_pub.publish(global_message);
          goal_map[map_type].restart();
          can_send_next_goal = true;
          changed_phase = true;
        }
        break;
      }
      }
      planner.sendEmergencyStatus();

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
    }
    loop_rate.sleep();
  }
}
