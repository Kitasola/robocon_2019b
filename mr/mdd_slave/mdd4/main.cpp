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
constexpr int RANGE = 512;
constexpr PinName ENCODER_PIN[NUM_ENCODER_PORT][2] = {
    {PA_0, PA_4}, {PA_1, PA_3}, {PA_8, PA_7}, {PB_6, PA_11}};
constexpr double diameter = 101.6; //直径

double goal_speed_3;
double data;

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

int goal_tray_point = 0, current_point = 0;
bool loadTray(int cmd, int rx_data, int &tx_data) {
  goal_tray_point = rx_data;
  tx_data = current_point;
  return true;
}

double current_speed_3;
bool d3_speed(int cmd, int rx_data, int &tx_data) {
  goal_speed_3 = (double)rx_data / 100;
  tx_data = current_speed_3;
  return true;
}

int main() {
  slave.addCMD(255, safe);
  slave.addCMD(20, loadTray);
  slave.addCMD(72, d3_speed);
  constexpr int TARY_MOTOR_ID = 0, MAX_TARY_MOTOR_SPEED = -100; // 下向き
  DigitalIn slit(PA_0);
  slit.mode(PullUp);
  int current_slit = 0, prev_slit = 0;
  constexpr int LIGHT = 1, DARK = 0;
  DigitalIn limit_lower(PA_11);
  limit_lower.mode(PullUp);
  int phase = 0, current_tray_point = 0;

  constexpr int motor_3 = 1;
  RotaryInc rotary_inc_3(PA_8, PA_7, 512, 1);

  PidPosition pid_3 = PidPosition(30, 0, 0, 0);
  while (true) {

    double get_speed_3 = rotary_inc_3.getSpeed();
    current_speed_3 = get_speed_3 * diameter * M_PI / 1000;
    spinMotor(motor_3, pid_3.control((double)goal_speed_3, current_speed_3));

    prev_slit = current_slit;
    current_slit = slit.read();

    switch (phase) {
    case 0:
      if (current_slit == LIGHT && prev_slit == DARK) {
        ++current_tray_point;
      }
      if (limit_lower.read() == 1) {
        current_tray_point = 7;
        if (goal_tray_point == 8) {
          goal_tray_point = 0;
          phase = 1;
        }
      }
      break;
    case 1:
      if (current_slit == DARK && prev_slit == LIGHT) {
        --current_tray_point;
      }
      if (current_tray_point == 0) {
        phase = 0;
      }
      break;
    }
    if (current_tray_point < goal_tray_point) {
      spinMotor(TARY_MOTOR_ID, MAX_TARY_MOTOR_SPEED);
    } else if (current_tray_point > goal_tray_point) {
      spinMotor(TARY_MOTOR_ID, -MAX_TARY_MOTOR_SPEED);
    } else {
      spinMotor(TARY_MOTOR_ID, 0);
    }
    current_point = current_tray_point;
  }
}
