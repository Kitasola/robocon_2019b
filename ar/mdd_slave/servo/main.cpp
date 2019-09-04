#include <mbed.h>
#include <rotary_inc.hpp>
#include <scrp_slave.hpp>

ScrpSlave slave(PA_9, PA_10, PA_12, SERIAL_TX, SERIAL_RX, 0x0803e000);

constexpr int NUM_PORT = 5;
// 0: Motor, 1: Encoder, 2: Other
constexpr int PORT_FUNCTION[NUM_PORT] = {0, 0, 0, 0, 0};

constexpr int NUM_MOTOR_PORT = 4;
constexpr int MAX_PWM = 250;
constexpr float PERIOD = 1 / 4000.0;
constexpr PinName MOTOR_PIN[NUM_MOTOR_PORT][3] = {{PB_0, PB_1, PB_3},
                                                  {PA_1, PA_3, PB_4},
                                                  {PA_8, PA_7, PB_5},
                                                  {PB_6, PA_11, PB_7}};
PwmOut *motor_pwm[NUM_MOTOR_PORT][2];
DigitalOut *motor_led[NUM_MOTOR_PORT];

constexpr int NUM_ENCODER_PORT = 4;
constexpr int RANGE = 200;
constexpr PinName ENCODER_PIN[NUM_ENCODER_PORT][2] = {
    {PA_0, PA_4}, {PA_1, PA_3}, {PA_8, PA_7}, {PB_6, PA_11}};
RotaryInc *rotary[NUM_ENCODER_PORT];

float map(float value, float from_low, float from_high, float to_low,
          float to_high) {
  if (value > from_high) {
    value = from_high;
  } else if (value < from_low) {
    value = from_low;
  }
  return value * (to_high - to_low) / (from_high - from_low);
}

bool spinMotor(int id, int value) {
  if (value == 0) {
    motor_pwm[id][0]->write(0);
    motor_pwm[id][1]->write(0);
    motor_led[id]->write(0);
  } else if (0 < value) {
    motor_pwm[id][0]->write(map(value, -MAX_PWM, MAX_PWM, -1.0, 1.0));
    motor_pwm[id][1]->write(0);
    motor_led[id]->write(1);
  } else {
    motor_pwm[id][0]->write(0);
    motor_pwm[id][1]->write(-map(value, -MAX_PWM, MAX_PWM, -1.0, 1.0));
    motor_led[id]->write(1);
  }
  return true;
}

bool safe(int cmd, int rx_data, int &tx_data) {
  for (int i = 0; i < 4; ++i) {
    spinMotor(i, 0);
  }
  return true;
}

bool spinMotor(int cmd, int rx_data, int &tx_data) {
  return spinMotor(cmd - 2, rx_data);
}

int main() {
  for (int i = 0; i < 4; ++i) {
    switch (PORT_FUNCTION[i]) {
    case 0:
      motor_pwm[i][0] = new PwmOut(MOTOR_PIN[i][0]);
      motor_pwm[i][0]->period(PERIOD);
      motor_pwm[i][1] = new PwmOut(MOTOR_PIN[i][1]);
      motor_pwm[i][1]->period(PERIOD);
      motor_led[i] = new DigitalOut(MOTOR_PIN[i][2]);
      break;
    case 1:
      rotary[i] = new RotaryInc(ENCODER_PIN[i][0], ENCODER_PIN[i][1], RANGE, 1);
      break;
    }
  }
  slave.addCMD(2, spinMotor);
  slave.addCMD(3, spinMotor);
  slave.addCMD(4, spinMotor);
  slave.addCMD(5, spinMotor);
  slave.addCMD(255, safe);
  while (true) {
  }
}
