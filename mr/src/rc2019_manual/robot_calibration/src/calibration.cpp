#include<ros/ros.h>
#include<std_msgs/String.h>
#include<three_omuni/button.h>

const unsigned int BIT_FLAG_GYRO = (1 << 0);
const unsigned int BIT_FLAG_ARM = (1 << 1);
const unsigned int ALL_CALIBRATION = BIT_FLAG_GYRO | BIT_FLAG_ARM;
unsigned int robot_status = 0;

void answer_callback(const std_msgs::String &file_receive){
    if(file_receive.data == "complete_gyro"){
        robot_status |= BIT_FLAG_GYRO;
    }
    if(file_receive.data == "complete_arm"){
        robot_status |= BIT_FLAG_ARM;
    }
}

bool flag_calibration = false;

void joy_callback(const three_omuni::button &button){
    button.calibration == 1 ? flag_calibration = true : flag_calibration = flag_calibration;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "robot_calibtarion");
    ros::NodeHandle n;
    ros::Publisher calibration_pub = n.advertise<std_msgs::String>("calibration", 10);
    ros::Subscriber calibration_sub = n.subscribe("calibration_answer", 30, answer_callback);
    ros::Subscriber controller_sub = n.subscribe("controller_info", 10, joy_callback);
    ros::Rate loop_rate(100);

    while(ros::ok()){
	if(flag_calibration == true){
           std_msgs::String file_send;
           if(robot_status == 0) file_send.data = "gyro";
           if(robot_status == BIT_FLAG_GYRO) file_send.data = "arm";
           if(robot_status == ALL_CALIBRATION) file_send.data = "complete";
	   calibration_pub.publish(file_send);
	}
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

