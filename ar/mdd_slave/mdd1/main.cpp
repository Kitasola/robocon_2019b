#include <mbed.h>
#include <rotary_inc.hpp>
#include <scrp_slave.hpp>

ScrpSlave slave(PA_9, PA_10, PA_12, SERIAL_TX, SERIAL_RX, 0x0803e000);

constexpr int NUM_PORT = 5;
// 0: Motor, 1: Encoder, 2: Other
constexpr int PORT_FUNCTION[NUM_PORT] = {0, 2, 2, 2, 0};

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

bool spinMotor(int cmd, int rx_data, int &tx_data) {
  return spinMotor(cmd - 2, rx_data);
}

bool safe(int cmd, int rx_data, int &tx_data) {
  for (int i = 0; i < 4; ++i) {
    if (PORT_FUNCTION[i] == 4) {
      spinMotor(i, 0);
    }
  }
  return true;
}

PwmOut towel[2] = {PwmOut(PA_3), PwmOut(PA_1)};
constexpr float MIN_SERVO_PULSE = 0.5e-3;
constexpr float MAX_SERVO_PULSE = 2.4e-3;
float servoDegreeToPulse(int degree) {
  return map(degree, 0, 180, MIN_SERVO_PULSE, MAX_SERVO_PULSE) +
         MIN_SERVO_PULSE;
}

constexpr int OFFSET_ANGLE[2] = {5, 13};
bool dryTowel(int cmd, int rx_data, int &tx_data) {
  towel[0].pulsewidth(servoDegreeToPulse(rx_data + OFFSET_ANGLE[0]));
  towel[1].pulsewidth(servoDegreeToPulse(180 - rx_data + OFFSET_ANGLE[1]));
  return true;
}

DigitalIn hanger_limit[2] = {DigitalIn(PB_0), DigitalIn(PA_0)};
bool serveHanger(int cmd, int rx_data, int &tx_data) {
  int speed = rx_data;
  if (speed > 0 && hanger_limit[0].read()) {
    speed = 0;
  } else if (speed < 0 && hanger_limit[1].read()) {
    speed = 0;
  }
  tx_data = speed;
  spinMotor(4, speed);
  spinMotor(5, speed);
  return true;
}

int main() {
  if (PORT_FUNCTION[0] == 0) {
    motor_pwm[0][0] = new PwmOut(MOTOR_PIN[0][0]);
    motor_pwm[0][0]->period(PERIOD);
    motor_pwm[0][1] = new PwmOut(MOTOR_PIN[0][1]);
    motor_pwm[0][1]->period(PERIOD);
    motor_led[0] = new DigitalOut(MOTOR_PIN[0][2]);
    slave.addCMD(2, spinMotor);
  }
  if (PORT_FUNCTION[1] == 0) {
    rotary[0] = new RotaryInc(ENCODER_PIN[0][0], ENCODER_PIN[0][1], RANGE, 1);
  }
  for (int i = 2; i < NUM_PORT; ++i) {
    switch (PORT_FUNCTION[i]) {
    case 0:
      motor_pwm[i - 1][0] = new PwmOut(MOTOR_PIN[i - 1][0]);
      motor_pwm[i - 1][0]->period(PERIOD);
      motor_pwm[i - 1][1] = new PwmOut(MOTOR_PIN[i - 1][1]);
      motor_pwm[i - 1][1]->period(PERIOD);
      motor_led[i - 1] = new DigitalOut(MOTOR_PIN[i - 1][2]);
      slave.addCMD(i + 1, spinMotor);
      break;
    case 1:
      rotary[i - 1] =
          new RotaryInc(ENCODER_PIN[i - 1][0], ENCODER_PIN[i - 1][1], RANGE, 1);
      break;
    }
  }
  slave.addCMD(255, safe);

  slave.addCMD(10, dryTowel);
  slave.addCMD(20, serveHanger);
  while (true) {
  }
}
