#include<ros/ros.h>
#include"/home/ubuntu/arrc_utility/raspi/include/pigpiod.hpp"
#include<cmath>
#include"motor_serial/motor_serial.h"
#include<iostream>
#include<three_omuni/button.h>
#include<std_msgs/Float64.h>

using std::cout;
using std::endl;
using std::cos;

float robot_angle = 0;
float robot_speed = 0;
float gyro_angle = 0;

void joy_callback(const three_omuni::button &move_info){
    robot_angle = move_info.move_angle;
    robot_speed = move_info.move_speed;
    //cout << robot_angle << ":" << robot_speed << endl;
}

void gyro_callback(const std_msgs::Float64 &gyro_info){
    gyro_angle = gyro_info.data;
}
    
void change_speed(float speed, float angle, float *wheel_control){
    float speed_x = -1 * speed * cos(angle);
    float speed_y = speed * sin(angle);
    cout << speed_x << ", " << speed_y << endl;
    wheel_control[0] = ((-1 * speed_x * cos(M_PI / 3)) + (speed_y * sin(M_PI / 3))) / 7; 
    wheel_control[1] = (-1) * ((-1 * speed_x * cos(M_PI / 3)) - (speed_y * sin(M_PI / 3))) / 4;
    wheel_control[2] = (-1) * speed_x / 4;
    cout << wheel_control[0] << ", " << wheel_control[1] << ", " << wheel_control[2] << endl;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "omuni");
    ros::NodeHandle n;
    ros::Subscriber controller_sub = n.subscribe("controller_info", 10, joy_callback);
    ros::Subscriber gyro_sub = n.subscribe("gyro_info", 10, gyro_callback);
    ros::ServiceClient motor_speed = n.serviceClient<motor_serial::motor_serial>("motor_info");
    motor_serial::motor_serial srv;
    //this parameter will be dicided later
    constexpr int WHEEL_ID[3] = {2, 2, 2};
    constexpr int WHEEL_CMD[3] = {2, 3, 4};
    float wheel_control[3] = {};
    ros::Rate loop_rate(10);

    while(ros::ok()){
        change_speed(robot_speed, robot_angle, wheel_control);
        for(int i = 0; i < 3; ++i){
            srv.request.id = (unsigned int)WHEEL_ID[i];
	    srv.request.cmd = (unsigned int)WHEEL_CMD[i];
            srv.request.data = (int)wheel_control[i];
	    motor_speed.call(srv); 
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
