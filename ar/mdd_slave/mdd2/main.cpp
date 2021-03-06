#include <mbed.h>
#include <pid.hpp>
#include <rotary_inc.hpp>
#include <scrp_slave.hpp>

using namespace arrc;

ScrpSlave slave(PA_9, PA_10, PA_12, SERIAL_TX, SERIAL_RX, 0x0803e000);

constexpr int NUM_PORT = 5;

constexpr int NUM_MOTOR_PORT = 4;
constexpr int MAX_PWM = 250;
constexpr double MAX_PWM_MBED = 0.95;
constexpr float PERIOD = 1 / 1000.0;
constexpr PinName MOTOR_PIN[NUM_MOTOR_PORT][3] = {{PB_0, PB_1, PB_3},
                                                  {PA_1, PA_3, PB_4},
                                                  {PA_8, PA_7, PB_5},
                                                  {PB_6, PA_11, PB_7}};

constexpr int NUM_ENCODER_PORT = 4;
constexpr int RANGE = 200;
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

constexpr int NUM_TOWEL = 2;
PwmOut towel[NUM_TOWEL] = {PwmOut(PA_3), PwmOut(PA_1)};
constexpr float MIN_SERVO_PULSE = 0.5e-3;
constexpr float MAX_SERVO_PULSE = 2.4e-3;
float servoDegreeToPulse(int degree) {
  return map(degree, 0, 180, MIN_SERVO_PULSE, MAX_SERVO_PULSE);
}

constexpr int OFFSET_ANGLE[NUM_TOWEL] = {77, 93};
int towel_goal_angle[NUM_TOWEL] = {};
bool dryTowel(int cmd, int rx_data, int &tx_data) {
  towel_goal_angle[0] = rx_data + OFFSET_ANGLE[0];
  towel_goal_angle[1] = -rx_data + OFFSET_ANGLE[1];
  return true;
}

bool dryTowelServo(int cmd, int rx_data, int &tx_data) {
  towel_goal_angle[cmd - 11] = rx_data;
  return true;
}

int goal_two_hight = 0;          // [mm]
int dummy_current_two_hight = 0; // [mm]
bool setTwoHigh(int cmd, int rx_data, int &tx_data) {
  // rx_data, tx_data [cm]
  goal_two_hight = rx_data * 10;
  tx_data = dummy_current_two_hight / 10;
  return true;
}

int two_hight_current[2] = {};
bool checkTwoRegister(int cmd, int rx_data, int &tx_data) {
  if (rx_data >= 10) {
    tx_data = two_hight_current[rx_data - 10] / 10;
  } else {
    tx_data = two_hight_current[rx_data] % 10;
  }
  return true;
}

int two_stage_speed[2] = {};
bool checkTwoVelocity(int cmd, int rx_data, int &tx_data) {
  tx_data = two_stage_speed[rx_data];
  return true;
}

bool safe(int cmd, int rx_data, int &tx_data) {
  for (int i = 0; i < 2; ++i) {
    two_stage_speed[i] = 0;
  }
  return true;
}

int main() {
  slave.addCMD(255, safe);
  slave.addCMD(4, spinMotor);
  slave.addCMD(5, spinMotor);

  slave.addCMD(10, dryTowel);
  slave.addCMD(11, dryTowelServo);
  slave.addCMD(12, dryTowelServo);
  slave.addCMD(30, setTwoHigh);
  slave.addCMD(33, checkTwoRegister);
  slave.addCMD(34, checkTwoVelocity);

  constexpr int NUM_TWO_REGISTER = 2;
  AnalogIn two_register[NUM_TWO_REGISTER] = {AnalogIn(PB_0),
                                             AnalogIn(PA_0)}; // right, left
  constexpr int TWO_STAGE_ID[NUM_TWO_REGISTER] = {2, 3};
  constexpr float TWO_REGISTER_MULTI[NUM_TWO_REGISTER] = {
      -720 / (210.0 / 255) * 5 / 3.3,
      720 / (210.0 / 255) * 5 / 3.3}; // mmへの変換倍率
  constexpr int TOW_STAGE_OFFSET[NUM_TWO_REGISTER] = {-1196, 127};
  constexpr double TWO_STAGE_DOWN = 0.5;
  PidPosition two_motor[NUM_TWO_REGISTER] = {PidPosition(5.0, 0, 0, 0),
                                             PidPosition(5.0, 0, 0, 0)};
  while (true) {
    for (int i = 0; i < NUM_TWO_REGISTER; ++i) {
      two_hight_current[i] =
          two_register[i].read() * TWO_REGISTER_MULTI[i] - TOW_STAGE_OFFSET[i];
      two_stage_speed[i] =
          two_motor[i].control(goal_two_hight, two_hight_current[i]);
      if (goal_two_hight < two_hight_current[i]) {
        two_stage_speed[i] *= TWO_STAGE_DOWN;
      }
      spinMotor(TWO_STAGE_ID[i], two_stage_speed[i]);
    }
    dummy_current_two_hight = (two_hight_current[0] + two_hight_current[1]) / 2;
    for (int i = 0; i < NUM_TOWEL; ++i) {
      towel[i].pulsewidth(servoDegreeToPulse(towel_goal_angle[i]));
    }
    wait(0.01);
  }
}
