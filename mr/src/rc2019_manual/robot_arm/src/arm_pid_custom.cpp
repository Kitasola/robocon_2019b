#include<ros/ros.h>
#include<rotary_inc.hpp>
#include<motor_serial.hpp>
#include<pigpiod.hpp>
#include"motor_serial/motor_serial.h"
#include<std_msgs/Float64MultiArray.h>

using ros::RotaryInc;
#define rep(i, n) for(int (i) = 0; (i) < (n); ++(i))
constexpr int resolution = 512;
constexpr int multiplication = 1;
constexpr double angle_gain[3] = {1, 1, 1};
constexpr int FREQ = 100;
double goal_angle[2];

class PID{
    public:
        PID(const double *gain, const int freq, const int resolution);
        void PidUpdate(double goal, double now, double prev);
        double Get();
    private:
        void Defferential();
        void Integral();
	void Calcurate();
        double Kp;
        double Ki;
        double Kd;
        int FREQ;
        double goal_value = 0;
        double now_value = 0;
	double prev_value = 0;
        double answer_value = 0;
        double integral_value = 0;
        double defferential_value = 0;
};

PID::PID(const double *gain, const int freq, const int resolution){
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

void arm_callback(const std_msgs::Float64MultiArray &msg){
    goal_angle[0] = msg.data[0] * (180 / M_PI);
    goal_angle[1] = msg.data[1] * (180 / M_PI);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "arm_pid");
    ros::NodeHandle n;
    ros::Subscriber angle_sub = n.subscribe("arm_angle", 10, arm_callback);
    ros::ServiceClient arm_client = n.serviceClient<motor_serial::motor_serial>("robot_arm");
    RotaryInc rotary[2] = {
        RotaryInc(22, 10, 1), RotaryInc(9, 11, 1)
    };
    double rotational_speed[2];
    double prev_rotational_speed[2];
    double final_angle[2] = {};
    float integral_motor[2] = {};
    float defferential_motor[2] = {};
    int ARM_ID[2] = {1, 1};
    int ARM_CMD[2] = {2, 3};
    constexpr float FREQ = 100.0;
    motor_serial::motor_serial srv;
    PID angle[2] = {PID(angle_gain, FREQ, resolution),
		    PID(angle_gain, FREQ, resolution)};
    ros::Rate loop_rate(100);

    while(ros::ok()){
        rotational_speed[0] = (360) * (1) * ((double)rotary[0].get() / (resolution * multiplication));
        rotational_speed[1] = (360) * (1) * ((double)rotary[1].get() / (resolution * multiplication));
        rep(i, 2){
	    angle[i].PidUpdate(goal_angle[i], rotational_speed[i], prev_rotational_speed[i]);
	    final_angle[i] = angle[i].Get();
	    prev_rotational_speed[i] = rotational_speed[i];
	    srv.request.id = (unsigned int)ARM_ID[i];
	    srv.request.cmd = (unsigned int)ARM_CMD[i];
	    srv.request.data = (int)final_angle[i] * -1;
	    arm_client.call(srv);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

