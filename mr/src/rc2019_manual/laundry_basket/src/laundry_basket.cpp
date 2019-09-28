#include<ros/ros.h>
#include<three_omuni/button.h>
#include"motor_serial/motor_serial.h"

ros::ServiceClient basket;
motor_serial::motor_serial srv;

void controllerCallback(const three_omuni::button &button){
    if(button.laundry_case_open){
        srv.request.id = 2;
        srv.request.cmd = 10;
        srv.request.data = 2;
        basket.call(srv);
    }
    if(button.laundry_case_close){
        srv.request.id = 2;
        srv.request.cmd = 10;
        srv.request.data = -1;
        basket.call(srv);
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "laundry_basket");
    ros::NodeHandle n;
    ros::Subscriber controller_sub = n.subscribe("controller_info", 10, controllerCallback);
    basket = n.serviceClient<motor_serial::motor_serial>("laundry_basket");
    ros::spin();
    return 0;
}
