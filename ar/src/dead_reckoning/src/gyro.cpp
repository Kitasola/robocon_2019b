#include <GY521.hpp>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "gyro");
  ros::NodeHandle n;
  /* ros::Subscriber global_sub = */
  /*     n.subscribe("global_message", 1, checkGlobalMessage); */
  ros::Publisher wheel_gyro_pub =
      n.advertise<std_msgs::Float32>("wheel/yaw", 1);
  std_msgs::Float32 yaw;

  // Calibration
  /* while (!Pigpiod) { */
  /*   if (!ros::ok()) { */
  /*     return 0; */
  /*   } */
  /* } */
  ros::GY521 gyro(0x68, 2, 1000, 1.02);
  gyro.start();
  ROS_INFO_STREAM("Calibration Finish");

  constexpr int MAIN_FREQ = 500;
  constexpr int TOPIC_FREQ = 10;
  ros::Rate loop_rate(MAIN_FREQ);
  ros::Time start = ros::Time::now();

  while (ros::ok()) {
    ros::spinOnce();
    ros::Time now = ros::Time::now();
    double delta = now.toSec() - start.toSec();
    gyro.update();
    if (delta > 1 / TOPIC_FREQ) {
      yaw.data = gyro.yaw_;
      wheel_gyro_pub.publish(yaw);
      start = now;
    }
    loop_rate.sleep();
  }
}
