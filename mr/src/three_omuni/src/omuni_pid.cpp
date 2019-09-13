#include<ros/ros.h>
#include<pigpiod.hpp>
#include<cmath>
#include"motor_serial/motor_serial.h"
#include<iostream>
#include<three_omuni/button.h>
#include<std_msgs/Float64.h>
#include<std_msgs/Float64MultiArray.h>
#include<rotary_inc.hpp>

using std::cout;
using std::endl;
using std::cos;
using ros::RotaryInc;
using ros::Pigpiod;

double robot_angle = 0;
double robot_speed = 0;
double gyro_angle = 0;
double resolutional = 0;
double WHEEL_1[3] = {};
double WHEEL_2[3] = {};
double WHEEL_3[3] = {};

class PID{
	public:
		PID(const double *gain, const double freq, const int resolution);
		void PidUpdate(double goal, double now, double prev);
		double Get();
	private:
		void Defferential();
		void Integral();
		void Calcurate();
		double Kp;
		double Ki;
		double Kd;
		double FREQ;
		double goal_value = 0;
		double now_value = 0;
		double prev_value = 0;
		double answer_value = 0;
		double integral_value = 0;
		double defferential_value = 0;
};

PID::PID(const double *gain, const double freq, const int resolution){
	Kp = gain[0];
	Ki = gain[1];
	Kd = gain[2];
	FREQ = freq;
}

void PID::PidUpdate(double goal, double now, double prev){
	goal_value = goal;
	now_value = now;
	prev_value = prev;

	Defferential();
	Integral();
	Calcurate();
}

void PID::Defferential(){
	defferential_value = (now_value - prev_value) / (1 / FREQ);
}

void PID::Integral(){
	integral_value += now_value * (1 / FREQ);
}

void PID::Calcurate(){
	answer_value = Kp * (goal_value - now_value)
		- Kd * defferential_value;
}

double PID::Get(){
	return answer_value;
}

void joy_callback(const three_omuni::button &move_info){
	robot_angle = (double)move_info.move_angle;
	robot_speed = (double)move_info.move_speed;
	if(move_info.turn_left){
		resolutional = 100.0;
	}else if(move_info.turn_right){
		resolutional = -100.0;
	}
	//cout << robot_angle << ":" << robot_speed << endl;
}

void gyro_callback(const std_msgs::Float64 &gyro_info){
	gyro_angle = (double)gyro_info.data;
}

void gain_callback_wheel1(const std_msgs::Float64MultiArray &gain){
	WHEEL_1[0] = gain.data[0];
	WHEEL_1[1] = gain.data[1];
	WHEEL_1[2] = gain.data[2];
}
void gain_callback_wheel2(const std_msgs::Float64MultiArray &gain){
	WHEEL_2[0] = gain.data[0];
	WHEEL_2[1] = gain.data[1];
	WHEEL_2[2] = gain.data[2];
}
void gain_callback_wheel3(const std_msgs::Float64MultiArray &gain){
	WHEEL_3[0] = gain.data[0];
	WHEEL_3[1] = gain.data[1];
	WHEEL_3[2] = gain.data[2];
}
void change_speed(double speed, double angle, double *wheel_control, double resolutional){
	//resolutional_speedは回転成分
	double speed_x = -1 * speed * cos(angle - gyro_angle);
	double speed_y = speed * sin(angle - gyro_angle);
	double resolutional_speed = resolutional - gyro_angle;
	cout << speed_x << ", " << speed_y << endl;
	wheel_control[0] = ((-1 * speed_x * cos(M_PI / 3)) + (speed_y * sin(M_PI / 3))) + resolutional_speed;
	wheel_control[1] = ((-1 * speed_x * cos(M_PI / 3)) - (speed_y * sin(M_PI / 3))) + resolutional_speed;
	wheel_control[2] = speed_x + resolutional_speed;
	cout << wheel_control[0] << ", " << wheel_control[1] << ", " << wheel_control[2] << endl;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "omuni");
	ros::NodeHandle n;
	ros::Subscriber controller_sub = n.subscribe("controller_info", 10, joy_callback);
	ros::Subscriber gyro_sub = n.subscribe("gyro_info", 10, gyro_callback);
	ros::Subscriber gain_sub_wheel1 = n.subscribe("pid_gain", 10, gain_callback_wheel1);
	ros::Subscriber gain_sub_wheel2 = n.subscribe("pid_gain", 10, gain_callback_wheel2);
	ros::Subscriber gain_sub_wheel3 = n.subscribe("pid_gain", 10, gain_callback_wheel3);
	ros::ServiceClient motor_speed = n.serviceClient<motor_serial::motor_serial>("motor_info");
	motor_serial::motor_serial srv;
	//this parameter will be dicided later
	constexpr int WHEEL_ID[3] = {1, 1, 1};
	constexpr int WHEEL_CMD[3] = {3, 4, 5};
	//constexpr double WHEEL_1[3] = {0.1, 0.1, 0.1};
	//constexpr double WHEEL_2[3] = {0.1, 0.1, 0.1};
	//constexpr double WHEEL_3[3] = {0.1, 0.1, 0.1};
	constexpr int resolution = 512;
	constexpr int multiplication = 1;
	constexpr double FREQ = 100.0;
	double wheel_control[3] = {};
	double wheel_speed[3] = {};
	double pid_result[3] = {};
	double prev_speed[3] = {};
	RotaryInc rotary[3] = {RotaryInc(26, 19, 1), RotaryInc(6, 5, 1), RotaryInc(11, 9, 1)};
	PID speed_pid[3] = {PID(WHEEL_1, FREQ, resolutional), PID(WHEEL_2, FREQ, resolutional), PID(WHEEL_3, FREQ, resolutional)};
	ros::Rate loop_rate(100);

	while(ros::ok()){
		change_speed(robot_speed, robot_angle, wheel_control, resolutional);
		for(int i = 0; i < 3; ++i){
			//速さを求める式[m/s]
			wheel_speed[i] = ((((double)rotary[i].get() / (resolution * multiplication)) * 101.6) / 1000) / (1 / FREQ);
			speed_pid[i].PidUpdate((double)wheel_control[i], wheel_speed[i], prev_speed[i]);
			pid_result[i] = speed_pid[i].Get();
			srv.request.id = (unsigned int)WHEEL_ID[i];
			srv.request.cmd = (unsigned int)WHEEL_CMD[i];
			srv.request.data = (int)pid_result[i];
			motor_speed.call(srv);
			prev_speed[i] = wheel_speed[i];
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
