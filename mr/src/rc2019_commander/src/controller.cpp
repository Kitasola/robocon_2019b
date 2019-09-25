#include<ros/ros.h>
#include<cmath>
#include<sensor_msgs/Joy.h>
#include<rc2019_commander/button.h>
#include<std_msgs/Bool.h>
#include<iostream>
using std::cout;
using std::endl;
rc2019_commander::button data; //button message

double stick_x, stick_y, rotation_right_law, rotation_left_law, angle_move, velocity_move, speed;

void joy_callback(const sensor_msgs::Joy &joy_msg){
    stick_x = -1 * joy_msg.axes[0];
    stick_y = joy_msg.axes[1];
    data.move_angle = atan2(stick_y, stick_x); //calculation in radians
    speed = hypot(stick_x, stick_y) * 255;//caululation of speed of movement direction of robot
    if(speed > 255) speed = 255;
    data.move_speed = speed;
    //I have not put the rotation component yet
    data.calibration = joy_msg.buttons[16];
    data.arm_data = joy_msg.buttons[13];
    data.turn_right = joy_msg.buttons[9];
    data.turn_left = joy_msg.buttons[8];
}

void check_callback(const std_msgs::Bool &check);

//洗濯物置き場までのコントローラーの配置
void joy_control_fase1();

//タオル掛けのコントローラーの配置
void joy_control_fase2();

int main(int argc, char **argv){
    ros::init(argc, argv, "manual_controller");
    ros::NodeHandle n;
    ros::Publisher controller_pub = n.advertise<rc2019_commander::button>("controller_info", 30);
    ros::Subscriber check_sub = n.subscribe("controller_check", 10, check_callback);
    ros::Subscriber joy_sub = n.subscribe("joy", 10, joy_callback);
    ros::Rate loop_rate(100);
    while(ros::ok()){
        controller_pub.publish(data);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}


void check_callback(const std_msgs::Bool &check){
    if(check.data){
        joy_control_fase1();
    }else{
        joy_control_fase2();
    }
}
void joy_control_fase1(){
}
void joy_control_fase2(){
}
