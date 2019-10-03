#include<ros/ros.h>
#include"pigpiod.hpp"
#include<cmath>
#include"motor_serial/motor_serial.h"
#include<iostream>
#include<three_omuni/button.h>
#include<std_msgs/Float64.h>
#include<std_msgs/Int16.h>
using std::cout;
using std::endl;
using std::cos;

double robot_angle = 0;
double robot_speed = 0;
double robot_rotation_right = 0;
double robot_rotation_left = 0;
double robot_right = 0;
double robot_left = 0;
//double resolutional = 0;
double gyro_angle = 0;
bool speed_half = false;

/*
   void joy_callback(const three_omuni::button &move_info){
   robot_angle = move_info.move_angle;
   robot_speed = move_info.move_speed;
   robot_rotation_right = move_info.move_turn_right;
   robot_rotation_left = move_info.move_turn_left;
   move_info.speed_half == true ? speed_half == true : speed_half == false;
//cout << robot_angle << ":" << robot_speed << endl;
}
 */
void joy_callback(const three_omuni::button &move_info){
	robot_angle = (double)move_info.move_angle;
	robot_speed = (double)move_info.move_speed;
	robot_right = (double)move_info.move_turn_left;
	robot_left = -(double)move_info.move_turn_right;
	move_info.speed_half == true ? speed_half = true : speed_half = false;
	//cout << robot_angle << ":" << robot_speed << endl;
}

void gyro_callback(const std_msgs::Float64 &gyro_info){
	gyro_angle = gyro_info.data * (M_PI / 180);
}

void change_speed(double speed, double angle, double *wheel_control, double turn_right, double turn_left){
	//resolutional_speed�~A��~[~^転�~H~P�~H~F
	double true_angle = angle - gyro_angle;
	double speed_x, speed_y;
	//if(true_angle >= (M_PI / 2) && true_angle <= ((2 *M_PI) / 3)){
	//if(cos(gyro_angle) >= 0){
		 speed_x = -1 * speed * cos(true_angle);
	//}else{
		 //speed_x = speed * cos(true_angle);
	//}
	//double speed_x = -1 * speed * cos(angle);
	//if(true_angle > ((2 * M_PI) / 3) && true_angle < (M_PI / 2)){
	//if(sin(gyro_angle) >= 0){
		speed_y = speed * sin(true_angle);
	//}else{
		//speed_y = -1 * speed * sin(true_angle);
	//}
	//double speed_y = speed * sin(angle);
	double resolutional_speed = robot_right + robot_left;
	//cout << speed_x << ", " << speed_y << endl;
	wheel_control[0] = 1.2 * (((-1 * speed_x * cos(M_PI / 3)) + (speed_y * sin(M_PI / 3))) + resolutional_speed) / 3;
	//wheel_control[0] = 1.2 * (((-1 * speed_x * cos(M_PI / 3)) + (speed_y * sin(M_PI / 3))) + resolutional_speed) / 3;
	wheel_control[1] = 1.4 * (((-1 * speed_x * cos(M_PI / 3)) - (speed_y * sin(M_PI / 3))) + resolutional_speed) / 3;
	//wheel_control[1] = 1.4 * (((-1 * speed_x * cos(M_PI / 3)) - (speed_y * sin(M_PI / 3))) + resolutional_speed) / 3;
	wheel_control[2] = 1.1 * (speed_x + resolutional_speed) / 3;
	//wheel_control[2] = 1.1 * (speed_x + resolutional_speed) / 3;
	//cout << wheel_control[0] << ", " << wheel_control[1] << ", " << wheel_control[2] << endl;
//	if(resolutional != 0){
//		wheel_control[0] = resolutional;
//		wheel_control[1] = resolutional;
//		wheel_control[2] = resolutional;
//	}
}

/*    
      void change_speed(float speed, float angle, float *wheel_control, float rotation_right, float rotation_left){
      float speed_x = -1 * speed * cos(angle);
      float speed_y = speed * sin(angle);
      cout << speed_x << ", " << speed_y << endl;
      wheel_control[0] = ((-1 * speed_x * cos(M_PI / 3)) + (speed_y * sin(M_PI / 3))); 
      wheel_control[1] = 1.5 * ((-1 * speed_x * cos(M_PI / 3)) - (speed_y * sin(M_PI / 3)));
      wheel_control[2] = speed_x;
      if(rotation_right > 0){
      wheel_control[0] = -rotation_right;
      wheel_control[1] = -rotation_right;
      wheel_control[2] = -rotation_right;
      }
      if(rotation_left > 0){
      wheel_control[0] = rotation_left;
      wheel_control[1] = rotation_left;
      wheel_control[2] = rotation_left;
      }

      cout << wheel_control[0] << ", " << wheel_control[1] << ", " << wheel_control[2] << endl;
      }
 */
int main(int argc, char **argv){
	ros::init(argc, argv, "omuni");
	ros::NodeHandle n;
	ros::Subscriber controller_sub = n.subscribe("controller_info", 10, joy_callback);
	ros::Subscriber gyro_sub = n.subscribe("gyro_info", 10, gyro_callback);
	ros::ServiceClient motor_speed = n.serviceClient<motor_serial::motor_serial>("motor_info");
	ros::Publisher check_pub = n.advertise<std_msgs::Int16>("check_wheel", 10);
	motor_serial::motor_serial srv;
	//this parameter will be dicided later
	constexpr int WHEEL_ID[3] = {1, 1, 4};
	constexpr int WHEEL_CMD[3] = {2, 5, 3};
	double wheel_control[3] = {};
	ros::Rate loop_rate(100);

	while(ros::ok()){
		std_msgs::Int16 msg;
		change_speed(robot_speed, robot_angle, wheel_control, robot_right, robot_left);
		for(int i = 0; i < 3; ++i){
			if(speed_half == true){
				wheel_control[i] /= 2;
			}
			srv.request.id = (unsigned int)WHEEL_ID[i];
			srv.request.cmd = (unsigned int)WHEEL_CMD[i];
			srv.request.data = (int)wheel_control[i];
			//srv.request.data = 100;
			motor_speed.call(srv);
		}
		//msg.data = (int)srv.request.data;
		msg.data = wheel_control[2];
		check_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
