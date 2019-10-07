#include<ros/ros.h>
#include<std_msgs/String.h>
#include"motor_serial/motor_serial.h"
#include<three_omuni/button.h>

motor_serial::motor_serial srv;
ros::ServiceClient expansion_client;
bool controller = false;
void expansionCallback(const std_msgs::String &msg){
    if(controller == false){
        int data_now = 0;
        if(msg.data == "fase1") data_now = 10;
        if(msg.data == "fase2") data_now = 15;
        if(msg.data == "fase3") data_now = 20;
        srv.request.id = 2;
        srv.request.cmd = 4;
        srv.request.data = data_now;
        expansion_client.call(srv);
    }
}

int dummy;

void controllerCallback(const three_omuni::button &msg){
    static int data_expansion = 0;
    if(msg.expansion_up){
        srv.request.id = 2;
        srv.request.cmd = 73;
        srv.request.data = 1;
    }
    if(msg.expansion_down){
        srv.request.id = 2;
        srv.request.cmd = 73;
        srv.request.data = 2;
    }
    if(msg.expansion_up == false && msg.expansion_down == false){
	srv.request.id = 2;
        srv.request.cmd = 73;
        srv.request.data = 0;
    }
    expansion_client.call(srv);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "expansion");
    ros::NodeHandle n;
    ros::Subscriber expansion_sub = n.subscribe("controller_info", 10, controllerCallback);
    expansion_client = n.serviceClient<motor_serial::motor_serial>("robot_expansion");

    ros::spin();
    return 0;
}
