#include <mbed.h>
#include <rotary_inc.hpp>
#include <scrp_slave.hpp>

ScrpSlave slave(PA_9, PA_10, PA_12, SERIAL_TX, SERIAL_RX, 0x0803e000);

constexpr int NUM_PORT = 5;
// 0: Motor, 1: Encoder, 2: Other
constexpr int PORT_FUNCTION[NUM_PORT] = {0, 2, 2, 2, 2};

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

PwmOut towel[2] = {PwmOut(PA_3), PwmOut(PA_1)};
/* constexpr float MIN_SERVO_PULSE = 780e-6; */
/* constexpr float MAX_SERVO_PULSE = 2250e-6; */
constexpr float MIN_SERVO_PULSE = 0.5e-3;
constexpr float MAX_SERVO_PULSE = 2.4e-3;
float servoDegreeToPulse(int degree) {
  return map(degree, 0, 180, MIN_SERVO_PULSE, MAX_SERVO_PULSE) +
         MIN_SERVO_PULSE;
}

bool driveServo(int cmd, int rx_data, int &tx_data) {
  if (cmd != 13) {
    towel[cmd - 11].pulsewidth(servoDegreeToPulse(rx_data));
  } else {
    towel[0].pulsewidth(servoDegreeToPulse(rx_data));
    towel[1].pulsewidth(servoDegreeToPulse(180 - rx_data));
  }
  return true;
}

AnalogIn r(PA_0);
bool readRegister(int cmd, int rx_data, int &tx_data) {
  tx_data = 255 * r.read();
  return true;
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
  /* slave.addCMD(2, spinMotor); */
  // slave.addCMD(3, spinMotor);
  /* slave.addCMD(4, spinMotor); */
  /* slave.addCMD(5, spinMotor); */
  slave.addCMD(11, driveServo);
  slave.addCMD(12, driveServo);
  slave.addCMD(13, driveServo);
  slave.addCMD(14, readRegister);
  slave.addCMD(255, safe);
  while (true) {
  }
}
