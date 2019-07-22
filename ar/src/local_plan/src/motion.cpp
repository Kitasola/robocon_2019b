#include "../include/ptp.hpp"
#include <ros/ros.h>

using namespace std;

int main(int argc, char **argv) {
  ros::init(argc, argv, "motion_planner");
  ros::NodeHandle n;

  Ptp controller(&n, "wheel/goal_point", "wheel/robot_pose");

  constexpr double FREQ = 300;
  ros::Rate loop_rate(FREQ);

  while (ros::ok()) {
    ros::spinOnce();
    controller.sendNextGoal();
  }
}
