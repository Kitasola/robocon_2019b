#ifndef ARRC_PTP_HPP
#define ARRC_PTP_HPP
#include <cmath>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <queue>
#include <ros/ros.h>

namespace arrc {
class Ptp {

public:
  Ptp(ros::NodeHandle *n, const std::string &goal_point,
      const std::string &current_point) {
    goal_point_pub = n->advertise<geometry_msgs::Pose>(goal_point, 1);
    current_point_sub = n->subscribe(current_point, 1, &Ptp::getCurrent, this);
  }

  void addGoal(geometry_msgs::Pose2D goal_point) {
    using std::queue;
    goal_point_list.push(goal_point);
  }

  void addGoal(double x, double y, double theta) {
    using std::queue;
    geometry_msgs::Pose2D dummy_goal;
    dummy_goal.x = x;
    dummy_goal.y = y;
    dummy_goal.theta = theta;
    goal_point_list.push(dummy_goal);
  }

  bool sendNextGoal() {
    using std::queue;
    if (reached()) {
      if (!goal_point_list.empty()) {
        goal_point = goal_point_list.front();
        ROS_INFO_STREAM("Next Goal Point is " << goal_point.x << ", "
                                              << goal_point.y << ", "
                                              << goal_point.theta * 180 / M_PI);
        goal_point_list.pop();
        goal_point_pub.publish(goal_point);
        return true;
      }
    }
    return false;
  }

  /* void getCurrentPoint(const geometry_msgs::Pose &msgs) { */
  /* current_point.x = msgs.position.x; */
  /* current_point.y = msgs.position.y; */
  /* double x = msgs.orientation.x, w = msgs.orientation.w; */
  /* current_point.theta = atan2(2 * x * w, x * x - w * w); */
  void getCurrent(const geometry_msgs::Pose2D &msgs) {
    current_point.x = msgs.x;
    current_point.y = msgs.y;
    current_point.theta = msgs.theta;
  }

  bool reached() {
    return (hypot(goal_point.x - current_point.x,
                  goal_point.y - current_point.y) < ERROR_MAX);
  }

private:
  std::queue<geometry_msgs::Pose2D> goal_point_list;
  ros::Subscriber current_point_sub;
  ros::Publisher goal_point_pub;
  geometry_msgs::Pose2D goal_point = {}, current_point = {};
  constexpr static double ERROR_MAX = 10;
}; // namespace arrc
} // namespace arrc

#endif
