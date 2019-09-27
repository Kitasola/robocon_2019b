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
constexpr double angle_P[2] = {1, 2};
constexpr double angle_I[2] = {1, 2};
constexpr double angle_D[2] = {1, 2};
constexpr double rpm_P[2] = {1, 2};
constexpr double rpm_I[2] = {1, 2};
constexpr double rpm_D[2] = {1, 2};
constexpr int FREQ = 100;
double goal_angle[2];

class PID{
    public:
        PID(const double gain_p, const double gain_i, const double gain_d, const int freq, const int resolution);
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

PID::PID(const double gain_p, const double gain_i, const double gain_d, const int freq, const int resolution){
    Kp = gain_p;
    Ki = gain_i;
    Kd = gain_d;
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
    double rpm_now[2];
    double prev_rotational_speed[2];
    double rpm_prev[2];
    double final_angle[2] = {};
    double final_rpm[2] = {};
    double integral_motor[2] = {};
    double integral_rpm[2] = {};
    double defferential_motor[2] = {};
    double deffetential_rpm[2] = {};
    int ARM_ID[2] = {1, 1};
    int ARM_CMD[2] = {2, 3};
    constexpr float FREQ = 100.0;
    motor_serial::motor_serial srv;
    PID angle[2] = {PID(angle_P[0], angle_I[0], angle_D[0], FREQ, resolution),
		    PID(angle_I[1], angle_I[1], angle_D[1], FREQ, resolution)};
    PID rpm[2] = {PID(rpm_P[0], rpm_I[0], rpm_D[0], FREQ, resolution),
		  PID(rpm_P[1], rpm_I[1], rpm_D[1], FREQ, resolution)};
    ros::Rate loop_rate(100);

    while(ros::ok()){
        rotational_speed[0] = (360) * (1) * ((double)rotary[0].get() / (resolution * multiplication));
        rotational_speed[1] = (360) * (1) * ((double)rotary[1].get() / (resolution * multiplication));
        rep(i, 2){
	    angle[i].PidUpdate(goal_angle[i], rotational_speed[i], prev_rotational_speed[i]);
	    final_angle[i] = angle[i].Get();

	    rpm_now[i] = (rotational_speed[i] / 60) * 100;
	    rpm[i].PidUpdate(final_angle[i], rpm_now[i], rpm_prev[i]);
	    final_rpm[i] = rpm[i].Get();
	    prev_rotational_speed[i] = rotational_speed[i];
	    rpm_prev[i] = rpm_now[i];
	    srv.request.id = (unsigned int)ARM_ID[i];
	    srv.request.cmd = (unsigned int)ARM_CMD[i];
	    srv.request.data = (int)final_rpm[i] * -1;
	    arm_client.call(srv);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

