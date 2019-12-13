#include <GY521.hpp>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "gyro");
  ros::NodeHandle n;
  /* ros::Subscriber global_sub = */
  /*     n.subscribe("global_message", 1, checkGlobalMessage); */
  ros::Publisher global_pub =
      n.advertise<std_msgs::String>("global_message", 1);
  std_msgs::String message;
  ros::Publisher wheel_gyro_pub =
      n.advertise<std_msgs::Float32>("wheel/yaw", 1);
  std_msgs::Float32 yaw;

  // コート情報の取得
  std::string coat_color;
  n.getParam("/coat", coat_color);
  int first_yaw;
  n.getParam("/ar/start_yaw", first_yaw);
  if (coat_color == "blue") {
    first_yaw *= 1;
  } else if (coat_color == "red") {
    first_yaw *= -1;
  } else {
    first_yaw *= 1;
  }

  ros::GY521 gyro(0x68, 1, 1000, 1.02);
  gyro.start(first_yaw);
  message.data = "Calibration Finish";
  global_pub.publish(message);

  constexpr int MAIN_FREQ = 1000;
  constexpr int TOPIC_FREQ = 30;
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
