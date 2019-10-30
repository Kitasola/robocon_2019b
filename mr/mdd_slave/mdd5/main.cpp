#include <mbed.h>
#include <pid.hpp>
#include <rotary_inc.hpp>
#include <scrp_slave.hpp>

using namespace arrc;

ScrpSlave slave(PA_9, PA_10, PA_12, SERIAL_TX, SERIAL_RX, 0x0803e000);

constexpr int NUM_PORT = 5;
// 0: Motor, 1: Encoder, 2: Other

constexpr int NUM_MOTOR_PORT = 4;
constexpr int MAX_PWM = 250;
constexpr double MAX_PWM_MBED = 0.95;
constexpr float PERIOD = 1 / 1000.0;
constexpr PinName MOTOR_PIN[NUM_MOTOR_PORT][3] = {{PB_0, PB_1, PB_3},
                                                  {PA_1, PA_3, PB_4},
                                                  {PA_8, PA_7, PB_5},
                                                  {PB_6, PA_11, PB_7}};

constexpr int NUM_ENCODER_PORT = 4;
constexpr int RANGE = 256;
constexpr PinName ENCODER_PIN[NUM_ENCODER_PORT][2] = {
    {PA_0, PA_4}, {PA_1, PA_3}, {PA_8, PA_7}, {PB_6, PA_11}};

float map(float value, float from_low, float from_high, float to_low,
          float to_high) {
  if (value > from_high) {
    value = from_high;
  } else if (value < from_low) {
    value = from_low;
  }
  return (value - from_low) * (to_high - to_low) / (from_high - from_low) +
         to_low;
}

bool spinMotor(int id, int value) {
  DigitalOut motor_led = DigitalOut(MOTOR_PIN[id][2]);
  if (value == 0) {
    DigitalOut motor_off[2] = {DigitalOut(MOTOR_PIN[id][0]),
                               DigitalOut(MOTOR_PIN[id][1])};
    motor_off[0].write(0);
    motor_off[1].write(0);
    motor_led.write(0);
  } else if (0 < value) {
    PwmOut motor_on(MOTOR_PIN[id][0]);
    motor_on.period(PERIOD);
    DigitalOut motor_off(MOTOR_PIN[id][1]);

    motor_on.write(map(value, -MAX_PWM, MAX_PWM, -MAX_PWM_MBED, MAX_PWM_MBED));
    motor_off.write(0);
    motor_led.write(1);
  } else {
    PwmOut motor_on(MOTOR_PIN[id][1]);
    motor_on.period(PERIOD);
    DigitalOut motor_off(MOTOR_PIN[id][0]);

    motor_off.write(0);
    motor_on.write(-map(value, -MAX_PWM, MAX_PWM, -MAX_PWM_MBED, MAX_PWM_MBED));
    motor_led.write(1);
  }
  return true;
}

bool spinMotor(int cmd, int rx_data, int &tx_data) {
  return spinMotor(cmd - 2, rx_data);
}

bool safe(int cmd, int rx_data, int &tx_data) {
  for (int i = 0; i < 4; ++i) {
    spinMotor(i, 0);
  }
  return true;
}

double arm_current_angle[2];
int arm_goal_angle[2] = {};
bool getArmGoal(int cmd, int rx_data, int &tx_data) {
  arm_goal_angle[cmd % 2] = rx_data;
  tx_data = arm_current_angle[cmd % 2];
  return true;
}

double arm_raw_data[2] = {};
bool checkArmRegister(int cmd, int rx_data, int &tx_data) {
  tx_data = arm_raw_data[rx_data % 2] * 1000;
  return true;
}

int main() {
  slave.addCMD(255, safe);
  slave.addCMD(60, getArmGoal);
  slave.addCMD(61, getArmGoal);
  slave.addCMD(120, checkArmRegister);

  constexpr double ARM_ANGLE_MIN[2] = {35, 122.4}, ARM_ANGLE_MAX[2] = {95, 30},
                   ARM_REGISTER_LOW[2] = {0.775, 0.06},
                   ARM_REGISTER_HIGH[2] = {1.0, 0.402};
  AnalogIn arm_joint[2] = {AnalogIn(PB_0), AnalogIn(PA_0)}; // Shoulder, Elbo
  constexpr int ARM_MOTOR_ID[2] = {1, 2};
  PidVelocity arm_motor[2] = {
      PidVelocity(5.0 * 0.7, 5.0 * 1.2 / 0.5, 5.0 * 0.075 * 0.5, MAX_PWM),
      PidVelocity(5.0 * 0.7, 5.0 * 1.2 / 0.2, 5.0 * 0.075 * 0.4, MAX_PWM)};
  constexpr int ARM_ANGLE_ERROR[2] = {1, 3}, ARM_SPEED_MIN[2] = {0, 10};
  arm_goal_angle[0] = 80;
  arm_goal_angle[1] = 45;
  while (true) {
    for (int i = 0; i < 2; ++i) {
      arm_raw_data[i] = arm_joint[i].read();
      arm_current_angle[i] =
          map(arm_raw_data[i], ARM_REGISTER_LOW[i], ARM_REGISTER_HIGH[i],
              ARM_ANGLE_MIN[i], ARM_ANGLE_MAX[i]);
      double arm_control =
          arm_motor[i].control(arm_goal_angle[i], arm_current_angle[i]);
      if (fabs(arm_goal_angle[i] - arm_current_angle[i]) < ARM_ANGLE_ERROR[i]) {
        arm_control = 0;
      } else if (fabs(arm_control) < ARM_SPEED_MIN[i]) {
        arm_control = ARM_SPEED_MIN[i];
      }
      spinMotor(ARM_MOTOR_ID[i], arm_control);
    }
    wait(0.01);
  }
}
