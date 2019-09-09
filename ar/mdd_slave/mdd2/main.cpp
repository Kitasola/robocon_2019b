#include <mbed.h>
#include <rotary_inc.hpp>
#include <scrp_slave.hpp>

ScrpSlave slave(PA_9, PA_10, PA_12, SERIAL_TX, SERIAL_RX, 0x0803e000);

constexpr int NUM_PORT = 5;
// 0: Motor, 1: Encoder, 2: Other
constexpr int PORT_FUNCTION[NUM_PORT] = {2, 2, 0, 0, 2};

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
  return (value - from_low) * (to_high - to_low) / (from_high - from_low) +
         to_low;
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
    spinMotor(i, 0);
  }
  return true;
}

int goal_two_hight = 0; // [mm]
// cmで高さを指定する
bool setTwoHigh(int cmd, int rx_data, int &tx_data) {
  goal_two_hight = rx_data * 10;
  return true;
}

int two_max_velocity = 0; // PWM/s
bool setTwoVelocity(int cmd, int rx_data, int &tx_data) {
  two_max_velocity = rx_data;
  return true;
}

int two_max_accel = 0; // PWM/s^2
bool setTwoAccel(int cmd, int rx_data, int &tx_data) {
  two_max_accel = rx_data;
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
      motor_pwm[i][0] = new PwmOut(MOTOR_PIN[i][0]);
      motor_pwm[i][0]->period(PERIOD);
      motor_pwm[i][1] = new PwmOut(MOTOR_PIN[i][1]);
      motor_pwm[i][1]->period(PERIOD);
      motor_led[i] = new DigitalOut(MOTOR_PIN[i][2]);
      slave.addCMD(i + 1, spinMotor);
      break;
    case 1:
      rotary[i] = new RotaryInc(ENCODER_PIN[i][0], ENCODER_PIN[i][1], RANGE, 1);
      break;
    }
  }
  slave.addCMD(255, safe);

  slave.addCMD(30, setTwoHigh);
  slave.addCMD(31, setTwoVelocity);
  slave.addCMD(32, setTwoAccel);
  while (true) {
    constexpr int NUM_ELEVATE_MOTOR = 2;
    AnalogIn hight_register[NUM_ELEVATE_MOTOR] = {
        AnalogIn(PB_0), AnalogIn(PA_0)}; // right, left
    constexpr float REGISTER_MULTI = 10; // mmへの変換倍率
    constexpr int TOW_STAGE_OFFSET = 0;
    for (int 0; i < NUM_ELEVATE_MOTOR; ++i) {
      current_hight[i] =
          hight_register[i].read() * REGISTER_MULTI - TOW_STAGE_OFFSET;
    }
  }
}
