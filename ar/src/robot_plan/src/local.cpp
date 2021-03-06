#include <cmath>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <pid.hpp>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <vector>

inline double pow2(double x) { return x * x; }
double abs(double a) { return fabs(a); }

struct AccelMap {
  double position;
  double velocity;
};

class SVelocity {

public:
  SVelocity(ros::NodeHandle *n, const std::string username, int user_rate) {
    velocity_pub =
        n->advertise<geometry_msgs::Twist>(username + "/velocity", 1);
    goal_point_sub = n->subscribe(username + "/goal_point", 1,
                                  &SVelocity::getGoalPoint, this);
    goal_velocity_sub = n->subscribe(username + "/goal_velocity", 1,
                                     &SVelocity::getGoalVelocity, this);
    accel_max_sub =
        n->subscribe(username + "/accel_max", 1, &SVelocity::changeAccel, this);
    emergency_stop_sub = n->subscribe(username + "/emergency_stop", 1,
                                      &SVelocity::checkEmergency, this);
    robot_pose_sub =
        n->subscribe("robot_pose", 1, &SVelocity::getRobotPose, this);
    rate = user_rate * MAP_SCOPE;
    int start_x, start_y;
    // コート情報の取得
    std::string coat_color;
    n->getParam("/coat", coat_color);
    int coat;
    if (coat_color == "blue") {
      coat = 1;
    } else if (coat_color == "red") {
      coat = -1;
    } else {
      coat = 1;
    }
    n->getParam("/ar/start_x", start_x);
    n->getParam("/ar/start_y", start_y);
    AccelMap dummy;
    goal_point.x = dummy.position = coat * start_x;
    dummy.velocity = 0;
    velocity_map[0].push_back(dummy);
    goal_point.y = dummy.position = start_y;
    dummy.velocity = 0;
    velocity_map[1].push_back(dummy);
    for (int i = 0; i < 2; ++i) {
      map_id_max[i] = velocity_map[i].size();
    }
  }

  void getGoalPoint(const geometry_msgs::Pose2D &msg) {
    goal_point = msg;
    setParam();
  }
  void getGoalVelocity(const geometry_msgs::Twist &msg) { goal_velocity = msg; }
  void getRobotPose(const geometry_msgs::Pose2D &msg) { current_point = msg; }
  /* void getRobotPose(const geometry_msgs::Pose &msgs) { */
  /* current_point.x = msgs.position.x; */
  /* current_point.y = msgs.position.y; */
  /* double x = msgs.orientation.x, w = msgs.orientation.w; */
  /* current_point.theta = atan2(2 * x * w, x * x - w * w); */
  /* } */
  void changeAccel(const std_msgs::Int32 &msg) { ACCEL_MAX = msg.data; }
  void checkEmergency(const std_msgs::Bool &msg) {
    should_stop_emergency = msg.data;
  }

  void control() {
    if (should_stop_emergency) {
      send_twist.linear.x = send_twist.linear.y = send_twist.angular.z = 0;
      send_twist.angular.y = -1;
    } else {
      send_twist.angular.y = WHEEL_DEBUG_MODE;
      for (int i = 0; i < 2; ++i) {
        int search_id_min = map_id[i] - MAP_SEARCH_RANGE;
        int search_id_max = map_id[i] + MAP_SEARCH_RANGE;
        if (search_id_min < 0) {
          search_id_min = 0;
        }
        if (search_id_max > map_id_max[i]) {
          search_id_max = map_id_max[i];
        }

        double dummy_current_point;
        if (i == 0) {
          dummy_current_point = current_point.x;
        } else {
          dummy_current_point = current_point.y;
        }
        int shortest_distance = INT_MAX;
        double dummy_velocity;
        for (int j = search_id_min; j < search_id_max; ++j) {
          if (shortest_distance >
              abs((dummy_current_point - velocity_map[i].at(j).position))) {
            shortest_distance =
                abs(dummy_current_point - velocity_map[i].at(j).position);
            map_id[i] = j;
          }
        }
        map_id[i] += MAP_SCOPE;
        if (map_id[i] >= map_id_max[i] - 1) {
          map_id[i] = map_id_max[i] - 1;
        }
        dummy_velocity = velocity_map[i].at(map_id[i]).velocity;
        if (i == 0) {
          send_twist.linear.x =
              dummy_velocity +
              ROOT_FOLLOW *
                  (velocity_map[i].at(map_id[i]).position - current_point.x);
        } else {
          send_twist.linear.y =
              dummy_velocity +
              ROOT_FOLLOW *
                  (velocity_map[i].at(map_id[i]).position - current_point.y);
        }
      }

      if (goal_point.theta - current_point.theta > M_PI) {
        goal_point.theta -= 2 * M_PI;
      } else if (goal_point.theta - current_point.theta < -M_PI) {
        goal_point.theta += 2 * M_PI;
      }

      send_twist.angular.y = WHEEL_DEBUG_MODE;
      send_twist.angular.z = moment.control(goal_point.theta * 180 / M_PI,
                                            current_point.theta * 180 / M_PI);
      if (send_twist.angular.z > MAX_MOMENT) {
        send_twist.angular.z = MAX_MOMENT;
      } else if (send_twist.angular.z < -MAX_MOMENT) {
        send_twist.angular.z = -MAX_MOMENT;
      }
    }

    velocity_pub.publish(send_twist);
  }

  void setParam() {
    double dummy_distance =
        hypot(goal_point.x - current_point.x, goal_point.y - current_point.y);
    double angle =
        atan2(goal_point.y - current_point.y, goal_point.x - current_point.x);
    double dummy_goal_velocity =
        hypot(goal_velocity.linear.x, goal_velocity.linear.y);

    double dummy_velocity_max =
        sqrt((pow2(VELOCITY_MIN) + pow2(dummy_goal_velocity) +
              ACCEL_MAX * dummy_distance) /
             2);
    if (dummy_velocity_max > VELOCITY_MAX) {
      dummy_velocity_max = VELOCITY_MAX;
    }
    double velocity_max[2] = {dummy_velocity_max * cos(angle),
                              dummy_velocity_max * sin(angle)};
    double velocity_first[2] = {VELOCITY_MIN * cos(angle),
                                VELOCITY_MIN * sin(angle)};
    double velocity_final[2] = {goal_velocity.linear.x, goal_velocity.linear.y};
    for (int i = 0; i < 2; ++i) {
      if (velocity_first[i] > 0 && velocity_first[i] < velocity_final_prev[i]) {
        velocity_first[i] = velocity_final_prev[i];
      } else if (velocity_first[i] < 0 &&
                 velocity_first[i] > velocity_final_prev[i]) {
        velocity_first[i] = velocity_final_prev[i];
      }
      velocity_final_prev[i] = velocity_final[i];
    }
    double accel_max[2] = {ACCEL_MAX * cos(angle), ACCEL_MAX * sin(angle)};

    double distance[2] = {dummy_distance * abs(cos(angle)),
                          dummy_distance * abs(sin(angle))};
    double accel_time[2] = {}, const_time[2] = {}, decel_time[2] = {};
    for (int i = 0; i < 2; ++i) {
      if (distance[i] > ERROR_DISTANCE_MAX) {
        accel_time[i] =
            2 * abs((velocity_max[i] - velocity_first[i]) / accel_max[i]);
        const_time[i] = (abs(distance[i]) -
                         (2 * pow2(velocity_max[i]) -
                          (pow2(velocity_first[i]) + pow2(velocity_final[i]))) /
                             abs(accel_max[i])) /
                            abs(velocity_max[i]) +
                        accel_time[i];
        decel_time[i] =
            2 * abs((velocity_max[i] - velocity_final[i]) / accel_max[i]) +
            const_time[i];
      } else {
        accel_max[i] = const_time[i] = decel_time[i] = 0;
      }

      start_point = current_point;
      double delta_t = 1.0 / rate;
      double dummy_start;
      if (i == 0) {
        dummy_start = start_point.x;
      } else {
        dummy_start = start_point.y;
      }
      velocity_map[i].resize(0);

      if (distance[i] > ERROR_DISTANCE_MAX) {
        for (int j = 0; j * delta_t <= decel_time[i]; ++j) {
          double time = j * delta_t;
          AccelMap data;
          if (time < accel_time[i]) {
            data.position =
                accel_max[i] * pow2(accel_time[i]) / (8 * pow2(M_PI)) *
                    (cos(2 * M_PI / accel_time[i] * time) - 1) +
                accel_max[i] * pow2(time) / 4 + velocity_first[i] * time;
            data.velocity = -accel_max[i] * accel_time[i] / (4 * M_PI) *
                                sin(2 * M_PI / accel_time[i] * time) +
                            accel_max[i] * time / 2 + velocity_first[i];
          } else if (time < const_time[i]) {
            time -= accel_time[i];
            data.position =
                velocity_max[i] * time +
                pow2(velocity_max[i] - velocity_first[i]) / accel_max[i] +
                2 * velocity_first[i] * (velocity_max[i] - velocity_first[i]) /
                    accel_max[i];
            data.velocity = velocity_max[i];
          } else {
            time -= const_time[i];
            data.position =
                -accel_max[i] * pow2(decel_time[i] - const_time[i]) /
                    (8 * pow2(M_PI)) *
                    (cos(2 * M_PI / (decel_time[i] - const_time[i]) * time) -
                     1) -
                accel_max[i] * pow2(time) / 4 + velocity_max[i] * time +
                velocity_max[i] * (const_time[i] - accel_time[i]) +
                pow2(velocity_max[i] - velocity_first[i]) / accel_max[i] +
                2 * velocity_first[i] * (velocity_max[i] - velocity_first[i]) /
                    accel_max[i];
            data.velocity =
                accel_max[i] * (decel_time[i] - const_time[i]) / (4 * M_PI) *
                    sin(2 * M_PI / (decel_time[i] - const_time[i]) * time) -
                accel_max[i] * time / 2 + velocity_max[i];
          }
          data.position += dummy_start;
          velocity_map[i].push_back(data);
        }
      } else {
        AccelMap data;
        data.position = 0;
        data.velocity = 0;
        data.position += dummy_start;
        velocity_map[i].push_back(data);
      }
      map_id[i] = 0;
      map_id_max[i] = velocity_map[i].size();

      // Debug
      ROS_INFO_STREAM("Axis Num: " << i);
      ROS_INFO_STREAM("SVelocity Parameter: "
                      << "v_0 = " << velocity_first[i] << ", "
                      << "v_m = " << velocity_max[i] << ", "
                      << "v_f = " << velocity_final[i] << ", "
                      << "x = " << distance[i] << ", "
                      << "a_m = " << accel_max[i]);
      ROS_INFO_STREAM("Accel Time: " << accel_time[i] << ", " << const_time[i]
                                     << ", " << decel_time[i]);
      for (int j = 0; j < map_id_max[i]; ++j) {
        AccelMap data = velocity_map[i].at(j);
        ROS_INFO_STREAM(delta_t * j << ", " << data.position << ", "
                                    << data.velocity);
      }
    }
  }

  ~SVelocity() {
    send_twist.angular.y = -1;
    velocity_pub.publish(send_twist);
  }

private:
  int rate = 0;
  geometry_msgs::Pose2D current_point = {}, goal_point = {}, start_point = {};
  ros::Subscriber goal_point_sub, robot_pose_sub, goal_velocity_sub,
      accel_max_sub, emergency_stop_sub;
  ros::Publisher velocity_pub;
  geometry_msgs::Twist send_twist, goal_velocity;
  double velocity_final_prev[2] = {};
  constexpr static double ERROR_DISTANCE_MAX = 50, ERROR_ANGLE_MAX = 1.0;
  constexpr static double VELOCITY_MIN = 400, VELOCITY_MAX = 3000;
  double ACCEL_MAX = 500;
  constexpr static double ROOT_FOLLOW = 1.7;
  std::vector<AccelMap> velocity_map[2];
  constexpr static int MAP_SCOPE = 1, MAP_SEARCH_RANGE = 5 * MAP_SCOPE;
  int map_id[2] = {};
  int map_id_max[2] = {};
  bool should_stop_emergency = false;

  /* arrc::PidVelocity moment{12, 0, 0}; */
  arrc::PidVelocity moment{8, 0, 0};
  constexpr static int MAX_MOMENT = 0;

  constexpr static int WHEEL_DEBUG_MODE = 0;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "local_planner");
  ros::NodeHandle n;

  constexpr int FREQ = 15;
  SVelocity controller(&n, "wheel", FREQ);

  ros::Rate loop_rate(FREQ);
  while (ros::ok()) {
    ros::spinOnce();
    controller.control();
    loop_rate.sleep();
  }
}
