#include<iostream>
#include<cmath>
#include<ros/ros.h>
#include<std_msgs/Float64MultiArray.h>

using std::cout;
using std::endl;

std_msgs::Float64MultiArray angle;
std_msgs::Float64MultiArray angle_check;

constexpr int A = 408;
constexpr int B = 286;
constexpr double calibration_angle_1 = M_PI / 2;
constexpr double calibration_angle_2 = M_PI - (M_PI / 6);

float x, y;

void positionCallback(const std_msgs::Float64MultiArray &position);
void angle_calcurate(float *angle_1, float *angle_2);

int main(int argc, char **argv){
	ros::init(argc, argv, "robot_arm");
	ros::NodeHandle n;
	ros::Publisher robot_arm_pub = n.advertise<std_msgs::Float64MultiArray>("arm_angle", 10);
	ros::Publisher check_pub = n.advertise<std_msgs::Float64MultiArray>("check", 10);
	ros::Subscriber robot_arm_sub = n.subscribe("angle_info", 10, positionCallback);
	float angle_1 = 0, angle_2 = 0;
	angle_check.data.resize(2);
	angle.data.resize(2);
	ros::Rate loop_rate(100);

	while(ros::ok()){
		angle_calcurate(&angle_1, &angle_2);
		ROS_INFO("angle_1 = %lf", angle_1 * (180 / M_PI));
		ROS_INFO("angle_2 = %lf", angle_2 * (180 / M_PI));
		angle_check.data[0] = x;
		angle_check.data[1] = y;
		//angle.data[0] = (M_PI / 2) - angle_1;
		//angle.data[1] = angle_1 - angle_2;
		angle.data[0] = angle_1 - calibration_angle_1;
		angle.data[1] = angle_2 - calibration_angle_2;
		robot_arm_pub.publish(angle);
		check_pub.publish(angle_check);
		ros::spinOnce();
		loop_rate.sleep();
	}
}

void positionCallback(const std_msgs::Float64MultiArray &position){
	x = position.data[0];
	y = position.data[1];
}

void angle_calcurate(float *angle_1, float *angle_2){
	float r = sqrt(pow(x, 2) + pow(y, 2));
	*angle_2 = (float)acos(-1 * (pow(r, 2) - pow(A, 2) - pow(B, 2)) / (2 * A * B));
	float theta_1 = atan2(y, x);
	float theta_2 = acos(-1 * (pow(B, 2) - pow(A, 2) - pow(r, 2)) / (2 * A * r));
	*angle_1 = (float)theta_1 + theta_2;
        angle_check.data[0] = *angle_1;
}
