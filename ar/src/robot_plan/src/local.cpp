#include <cmath>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <pid.hpp>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <vector>

class SVelocity {

public:
  SVelocity(ros::NodeHandle *n, const std::string username, int rate) {
    velocity_pub =
        n->advertise<geometry_msgs::Twist>(username + "/velocity", 1);
    reach_goal_pub =
        n->advertise<geometry_msgs::Twist>(username + "/reach_goal", 1);
    goal_point_sub = n->subscribe(username + "goal_point", 1,
                                  &SVelocity::getGoalPoint, this);
    goal_velocity_sub = n->subscribe(username + "goal_velocity", 1,
                                     &SVelocity::getGoalVelocity, this);
    robot_pose_sub =
        n->subscribe("robot_pose", 1, &SVelocity::getRobotPose, this);
    rate_ = rate;
    loop_rate = new ros::Rate(rate);
  }

  void getGoalPoint(const geometry_msgs::Pose2D &msg) {
    goal_point = msg;
    setParam();
  }
  void getGoalVelocity(const geometry_msgs::Twist &msg) { goal_velocity = msg; }
  void getRobotPose(const geometry_msgs::Pose2D &msg) { current_point = msg; }

  void control() {
    if (hypot(goal_point.x - current_point.x, goal_point.y - current_point.y) <
        ERROR_DISTANCE_MAX) {
      std_msgs::Bool msgs;
      msgs.data = true;
      reach_goal_pub.publish(msgs);
    }

    /* send_twist.linear.x = velocity_map; */
    /* send_twist.linear.x = velocity_map; */
    send_twist.angular.z = -moment.control(goal_point.theta * 180 / M_PI,
                                           current_point.theta * 180 / M_PI);
    velocity_pub.publish(send_twist);
    loop_rate->sleep();
  }

  void setParam() {
    double dummy_distance =
        hypot(goal_point.x - current_point.x, goal_point.y - current_point.y);
    double angle =
        atan2(goal_point.y - current_point.y, goal_point.x - current_point.x);
    double dummy_goal_velocity =
        hypot(goal_velocity.linear.x, goal_velocity.linear.y);
    ROS_INFO_STREAM("Distance is " << dummy_distance << ", "
                                   << angle * 180 / M_PI);

    double dummy_velocity_max =
        ((VELOCITY_MIN + dummy_goal_velocity) / ACCEL_MAX +
         sqrt(pow((VELOCITY_MIN + dummy_goal_velocity) / ACCEL_MAX, 2) +
              4 * dummy_distance / ACCEL_MAX)) /
        (4 / ACCEL_MAX);
    if (dummy_velocity_max > VELOCITY_MAX) {
      dummy_velocity_max = VELOCITY_MAX;
    }
    double velocity_max[2] = {dummy_velocity_max * cos(angle),
                              dummy_velocity_max * sin(angle)};
    double velocity_first[2] = {VELOCITY_MIN * cos(angle),
                                VELOCITY_MIN * sin(angle)};
    for (int i = 0; i < 2; ++i) {
      if (velocity_first[i] < velocity_final_prev[i]) {
        velocity_first[i] = velocity_final_prev[i];
      }
    }
    double velocity_final[2] = {goal_velocity.linear.x, goal_velocity.linear.y};
    double accel_max[2] = {ACCEL_MAX * cos(angle), ACCEL_MAX * sin(angle)};

    double distance[2] = {dummy_distance * abs(cos(angle)),
                          dummy_distance * abs(sin(angle))};
    double accel_time[2] = {}, const_time[2] = {}, decel_time[2] = {};
    int accel_polar[2] = {};
    for (int i = 0; i < 2; ++i) {
      accel_time[i] =
          2 * abs(velocity_max[i] - velocity_first[i]) / accel_time[i];
      const_time[i] =
          (distance[i] -
           2 * velocity_max[i] *
               (2 * velocity_max[i] - velocity_first[i] - velocity_final[i]) /
               accel_max[i]) /
              velocity_max[i] +
          accel_time[i];
      decel_time[i] =
          2 * abs(velocity_max[i] - velocity_final[i]) / accel_time[i] +
          const_time[i];

      if (velocity_max[i] > velocity_first[i]) {
        accel_polar[i] = 1;
      } else {
        accel_polar[i] = -1;
      }
      ROS_INFO_STREAM("Velocity Parameter is " << i << ", " << velocity_first[i]
                                               << ", " << velocity_max[i]);
      ROS_INFO_STREAM("Accel Time Parameter is " << i << ", " << accel_time[i]
                                                 << ", " << const_time[i]
                                                 << ", " << decel_time[i]);
    }
    start_point = current_point;
    double delta_t = 1.0 / rate_;
    for (int time = 0; time * delta_t < decel_time[0]; ++time) {
      if (time * delta_t < accel_time[0]) {
      } else if (time * delta_t < const_time[0]) {
      } else {
      }
      /* velocity_map[0].push_back(); */
    }
  }

private:
  int rate_ = 0;
  ros::Rate *loop_rate;
  geometry_msgs::Pose2D current_point = {}, goal_point = {}, start_point = {};
  ros::Subscriber goal_point_sub, robot_pose_sub, goal_velocity_sub;
  ros::Publisher velocity_pub, reach_goal_pub;
  geometry_msgs::Twist send_twist, goal_velocity;
  double velocity_final_prev[2] = {};
  constexpr static double VELOCITY_MIN = 500, VELOCITY_MAX = 1000,
                          ACCEL_MAX = 1500;
  constexpr static double ERROR_DISTANCE_MAX = 10;
  std::vector<double> velocity_map[2];

  arrc::PidVelocity moment{10, 0, 0};
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "local_planner");
  ros::NodeHandle n;

  SVelocity controller(&n, "wheel/velocity", 300);

  while (ros::ok()) {
    ros::spinOnce();
    controller.control();
  }
}
