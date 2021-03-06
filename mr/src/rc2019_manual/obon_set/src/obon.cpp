#include "motor_serial/motor_serial.h"
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <three_omuni/button.h>
bool flag_hand = false;
bool flag_hand_prev = false;
bool flag_prev = false;
void controllerCallback(const three_omuni::button &msg) {
  // msg.hand == true ? flag_hand = true : flag_hand = false;
  if (msg.hand) {
    if (flag_prev == false) {
      if (flag_hand_prev) {
        flag_hand = false;
      } else {
        flag_hand = true;
      }
      flag_hand_prev = flag_hand;
    }
    flag_prev = true;
  } else {
    flag_prev = false;
  }
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_hand");
  ros::NodeHandle n;
  ros::Subscriber controller_sub =
      n.subscribe("controller_info", 100, controllerCallback);
  ros::ServiceClient robot_hand =
      n.serviceClient<motor_serial::motor_serial>("hand_info");
  ros::Publisher hand_pub = n.advertise<std_msgs::Int16>("check_pub", 10);
  motor_serial::motor_serial srv;
  ros::Rate loop_rate(1000);

  while (ros::ok()) {
    std_msgs::Int16 check;
    int data;
    // flag_hand == true ? data = 180 : data = 0;
    if (flag_hand) {
      data = 128;
    } else {
      data = 150;
    }
    srv.request.id = 6;
    srv.request.cmd = 40;
    srv.request.data = data;
    robot_hand.call(srv);
    ROS_INFO("%d", data);
    check.data = data;
    hand_pub.publish(check);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
