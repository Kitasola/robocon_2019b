#include <cmath>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <queue>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

class Ptp {
public:
  Ptp(ros::NodeHandle *n, const std::string usename) {
    goal_point_pub =
        n->advertise<geometry_msgs::Pose2D>(usename + "/goal_point", 1);
    reach_goal_sub =
        n->subscribe(usename + "/reach_goal", 1, &Ptp::checkReachGoal, this);
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

  bool is_reach_goal;
  void checkReachGoal(const std_msgs::Bool &msg) { is_reach_goal = msg.data; }
  void sendNextGoal() {
    using std::queue;
    if (!goal_point_list.empty()) {
      goal_point = goal_point_list.front();
      ROS_INFO_STREAM("Next Goal Point is " << goal_point.x << ", "
                                            << goal_point.y << ", "
                                            << goal_point.theta * 180 / M_PI);
      goal_point_list.pop();
      goal_point_pub.publish(goal_point);
      is_reach_goal = false;
    }
  }

private:
  std::queue<geometry_msgs::Pose2D> goal_point_list;
  ros::Subscriber reach_goal_sub;
  ros::Publisher goal_point_pub;
  geometry_msgs::Pose2D goal_point = {};
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "motion_planner");
  ros::NodeHandle n;
  Ptp planner(&n, "wheel");

  constexpr double FREQ = 300;
  ros::Rate loop_rate(FREQ);
  /* planner.addGoal(1000, 1000, 0); */
  planner.sendNextGoal();

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}
