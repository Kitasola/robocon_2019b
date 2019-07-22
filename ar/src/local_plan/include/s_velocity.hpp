#ifndef ARRC_SVELOCITY_HPP
#define ARRC_SVELOCITY_HPP
#include "../include/pid.hpp"
#include <cmath>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <queue>
#include <ros/ros.h>

namespace arrc {
class SVelocity {
public:
  SVelocity(ros::NodeHandle *n, const string &velocity, const string &goal) {
    velocity_pub = n->advertise<geometry_msgs::Twist>(velocity, 1);
    goal_sub = n->subscribe(goal, 1, &PTP::getGoalPoint, this);
    current_pub = n->subscribe(current_point, 1, &PTP::getCurrent, this);
    start = ros::Time::now();
  }

  void control() {
    ros::Time current = ros::Time::now();
    double duration = current.toSec() - start.toSec();
    double delta = current.toSec() - prev.toSec();
    bool is_accel_finish = true;
    for (int i = 0; i < 2; ++i) {
      if (duration < accel_time[i]) {
        velocity_goal[i] += accel_polar[i] * ACCEL_MAX *
                            sin(2 * ACCEL_MAX / velocity_diff[i] * duration) *
                            delta;
        is_accel_finish = false;
      } else if (duration < const_time[i]) {
        is_accel_finish = false;
      } else if (duration < decel_time[i]) {
        velocity_goal[i] -=
            accel_polar[i] * ACCEL_MAX *
            sin(2 * ACCEL_MAX / velocity_diff[i] * (duration - const_time[i])) *
            delta;
        is_accel_finish = false;
      }
    }

    if (is_accel_finish) {
      setParam();
    }

    goal_twist.linear.x = velocity_goal[0];
    goal_twist.linear.y = velocity_goal[1];
    goal_twist.angular.z = -moment.control(goal_point.theta * 180 / M_PI,
                                           current_point.theta * 180 / M_PI);
    velocity_pub.publish(goal_twist);
    prev = current;
  }

  void getGoal(const geometry_msgs::Pose2D msg) { goal_point = msg; }
  void getCurrent(const geometry_msgs::Pose2D &msg) { current_point = msg; }

  void setParam() {
    double length =
        hypot(goal_point.x - current_point.x, goal_point.y - current_point.y);
    double angle =
        atan2(goal_point.y - current_point.y, goal_point.x - current_point.x);
    ROS_INFO_STREAM("Distance is " << length << ", " << angle * 180 / M_PI);

    double velocity =
        sqrt(2 * ACCEL_MAX * length / M_PI + pow(VELOCITY_MIN, 2));
    if (velocity > VELOCITY_MAX) {
      velocity = VELOCITY_MAX;
    }
    velocity_max[0] = velocity * cos(angle);
    velocity_max[1] = velocity * sin(angle);
    velocity_min[0] = VELOCITY_MIN * cos(angle);
    velocity_min[1] = VELOCITY_MIN * sin(angle);
    for (int i = 0; i < 2; ++i) {
      velocity_goal[i] = velocity_min[i];
    }
    for (int i = 0; i < 2; ++i) {
      ROS_INFO_STREAM("Velocity Parameter is " << i << ", " << velocity_min[i]
                                               << ", " << velocity_max[i]);
    }

    double dummy_length[2] = {length * abs(cos(angle)),
                              length * abs(sin(angle))};
    for (int i = 0; i < 2; ++i) {
      velocity_diff[i] = abs(velocity_max[i] - velocity_min[i]);
      accel_time[i] = decel_time[i] = M_PI * velocity_diff[i] / 2 / ACCEL_MAX;
      const_time[i] = (dummy_length[i] -
                       2 * (pow(velocity_max[i], 2) - pow(velocity_min[i], 2)) *
                           M_PI / 4 / ACCEL_MAX) /
                      velocity_max[i];
      if (const_time[i] < 0) {
        const_time[i] = 0;
      }

      const_time[i] += accel_time[i];
      decel_time[i] += const_time[i];

      if (velocity_max[i] > velocity_goal[i]) {
        accel_polar[i] = 1;
      } else {
        accel_polar[i] = -1;
      }
      velocity_goal[i] = velocity_min[i];
      ROS_INFO_STREAM("Accel Time Parameter is " << i << ", " << accel_time[i]
                                                 << ", " << const_time[i]
                                                 << ", " << decel_time[i]);
    }
    start = ros::Time::now();
  }

private:
  geometry_msgs::Pose2D current_point = {}, goal_point = {};
  ros::Subscriber goal_sub, current_pub;
  ros::Publisher velocity_pub;
  ros::Time start, prev;
  geometry_msgs::Twist goal_twist;
  int accel_polar[2] = {};
  double velocity_goal[2] = {}, velocity_max[2] = {}, velocity_min[2] = {},
         velocity_diff[2] = {};
  double accel_time[2] = {}, const_time[2] = {}, decel_time[2] = {};
  constexpr static double VELOCITY_MIN = 500, VELOCITY_MAX = 1000,
                          ACCEL_MAX = 1500;

  PidVelocity moment{10, 0, 0};

} // namespace arrc

#endif
