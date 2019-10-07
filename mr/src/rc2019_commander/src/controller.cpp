#include <cmath>
#include <iostream>
#include <rc2019_commander/button.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
using std::cout;
using std::endl;
rc2019_commander::button data; // button message

double stick_x, stick_y, rotation_right_law, rotation_left_law, angle_move,
    velocity_move, speed;

namespace akashi {
double map(double x, double in_min, double in_max, double out_min,
           double out_max) {
  return (double)(x - in_min) * (out_max - out_min) / (in_max - in_min) +
         out_min;
}
}

void joy_callback(const sensor_msgs::Joy &joy_msg) {
  stick_x = joy_msg.axes[2];
  stick_y = joy_msg.axes[3];
  data.move_angle = atan2(stick_y, stick_x); // calculation in radians
  speed = hypot(stick_x, stick_y) *
          255; // caululation of speed of movement direction of robot
  if (speed > 255)
    speed = 255;
  data.move_speed = speed;
  data.move_turn_right = akashi::map(joy_msg.axes[13], 1, -1, 0, 255);
  data.move_turn_left = akashi::map(joy_msg.axes[12], 1, -1, 0, 255);
  data.move_arm_x = akashi::map(joy_msg.axes[0], 1, -1, -0.1, 0.1);
  data.move_arm_y = akashi::map(joy_msg.axes[1], -1, 1, -0.1, 0.1);
  // I have not put the rotation component yet
  data.calibration = joy_msg.buttons[16];
  data.arm_data_1 = joy_msg.buttons[5];
  data.arm_data_2 = joy_msg.buttons[7];
  data.turn_right = joy_msg.buttons[9];
  data.turn_left = joy_msg.buttons[8];
  data.expansion_up = joy_msg.buttons[4];
  data.expansion_down = joy_msg.buttons[6];
  data.speed_half = joy_msg.buttons[11];
  data.loading = joy_msg.buttons[14];
  data.shooting = joy_msg.buttons[15];
  data.hand = joy_msg.buttons[10];
  data.laundry_case_open = joy_msg.buttons[12];
  data.laundry_case_close = joy_msg.buttons[13];
  data.obon = joy_msg.buttons[3];
  // robot_arm position key(right key)
  if (joy_msg.buttons[4] && joy_msg.buttons[10]) {
    data.expansion_up_key = true;
  } else {
    data.expansion_up_key = false;
  }
  // robot_arm position key(left key)
  if (joy_msg.buttons[6] && joy_msg.buttons[10]) {
    data.expansion_down_key = true;
  } else {
    data.expansion_down_key = false;
  }
}

void check_callback(const std_msgs::Bool &check);

//洗濯物置き場までのコントローラーの配置
void joy_control_fase1();

//タオル掛けのコントローラーの配置
void joy_control_fase2();

int main(int argc, char **argv) {
  ros::init(argc, argv, "manual_controller");
  ros::NodeHandle n;
  ros::Publisher controller_pub =
      n.advertise<rc2019_commander::button>("controller_info", 50);
  ros::Subscriber check_sub =
      n.subscribe("controller_check", 10, check_callback);
  ros::Subscriber joy_sub = n.subscribe("joy", 10, joy_callback);
  ros::Rate loop_rate(1000);
  while (ros::ok()) {
    controller_pub.publish(data);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

void check_callback(const std_msgs::Bool &check) {
  if (check.data) {
    joy_control_fase1();
  } else {
    joy_control_fase2();
  }
}
void joy_control_fase1() {}
void joy_control_fase2() {}
