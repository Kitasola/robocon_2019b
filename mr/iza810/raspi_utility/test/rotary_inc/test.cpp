#include "../../include/pigpiod.hpp"
#include "../../include/rotary_inc.hpp"
#include <iostream>
#include <ros/ros.h>

using ros::RotaryInc;
using ros::Pigpiod;
using std::stoi;

int main(int argc, char *argv[]) {
  if (argc < 3) {
    ROS_WARN_STREAM("Too few arguments.\nex) ./test 3 4");
    return -1;
  }
  int multiplier = 1;
  if (argc > 3) {
    multiplier = stoi(argv[3]);
  }
  RotaryInc rotary(stoi(argv[1]), stoi(argv[2]), multiplier);

  ros::init(argc, argv, "RotaryInc_test");
  ros::NodeHandle n;
  ros::Time start = ros::Time::now();
  ros::Duration time;
  ros::Rate loop_rate(1000);

  while (ros::ok()) {
    time = ros::Time::now() - start;
    ROS_INFO_STREAM(time.sec << "." << time.nsec << ", " << rotary.get());
    loop_rate.sleep();
  }

  return 0;
}
