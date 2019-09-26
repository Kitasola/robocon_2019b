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
constexpr int RANGE = 512;
constexpr PinName ENCODER_PIN[NUM_ENCODER_PORT][2] = {
    {PA_0, PA_4}, {PA_1, PA_3}, {PA_8, PA_7}, {PB_6, PA_11}};
constexpr int diameter = 101.6; //直径

int goal_speed_3;

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

int load_mode;
bool loadLaundry(int cmd, int rx_data, int &tx_data) {
  load_mode = rx_data;
  return true;
}

int main() {
  slave.addCMD(4, spinMotor);
  slave.addCMD(255, safe);
  slave.addCMD(10, loadLaundry);

  // 洗濯物回収
  DigitalIn limit_front(PB_0);
  limit_front.mode(PullUp);
  DigitalIn limit_back(PB_1);
  limit_back.mode(PullUp);
  DigitalIn limit_stand(PA_);
  limit_stand.mode(PullUp);
  constexpr int MAX_LAUNDRY_SPEED = 100;
  int laundry_speed = 0;

  while (true) {
    switch (load_mode) {
    case -1:
      laundry_speed = -MAX_LAUNDRY_SPEED;
      if (limit_back.read() == 1) {
        laundry_speed = 0;
      }
      break;

    case 0:
      laundry_speed = 0;
      break;

    case 1:
      laundry_speed = MAX_LAUNDRY_SPEED;
      if (limit_front.read() == 1) {
        laundry_speed = 0;
      }
      break;

    case 2:
      if (limit_stand.read() == 1) {
        laundry_speed = -MAX_LAUNDRY_SPEED;
        if (limit_back.read() == 1) {
          laundry_speed = 0;
        }
      } else {
        laundry_speed = MAX_LAUNDRY_SPEED;
        if (limit_front.read() == 1) {
          laundry_speed = 0;
        }
      }
      break;
    }
    spinMotor(2, laundry_speed);
  }
}
