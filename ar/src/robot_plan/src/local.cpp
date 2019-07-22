#include "../include/s_velocity.hpp"
#include <ros/ros.h>

using namespace arrc;

int main(int argc, char **argv) {
  ros::init(argc, argv, "local_planner");
  ros::NodeHandle n;

  SVelocity controller(&n, "wheel/velocity", "wheel/goal_point",
                       "wheel/robot_pose");

  constexpr double FREQ = 300;
  ros::Rate loop_rate(FREQ);

  while (ros::ok()) {
    ros::spinOnce();
    controller.control();
  }
}
