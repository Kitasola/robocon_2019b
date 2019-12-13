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
  spinMotor(0, 0);
  spinMotor(1, 0);
  return true;
}

int goal_tray_point = 0, current_point = 0, phase = 0;
bool loadTray(int cmd, int rx_data, int &tx_data) {
  if (rx_data == -1) {
    phase = -1;
    goal_tray_point = 0;
  } else {
    phase = 0;
    goal_tray_point = rx_data;
  }
  tx_data = current_point;
  return true;
}

int main() {
  slave.addCMD(255, safe);
  slave.addCMD(3, spinMotor);
  slave.addCMD(20, loadTray);
  constexpr int TRAY_MOTOR_ID = 0, MAX_TRAY_MOTOR_SPEED = -100; // 下向き
  int tray_motor_polor = 0;
  DigitalIn slit(PA_0);
  slit.mode(PullUp);
  constexpr int LIGHT = 1, DARK = 0;
  int current_slit = LIGHT, prev_slit = LIGHT;
  DigitalIn limit_higher(PB_6);
  limit_higher.mode(PullUp);
  DigitalIn limit_lower(PA_11);
  limit_lower.mode(PullUp);
  DigitalOut limit_led_higher(PB_7);
  DigitalOut limit_led_lower(PB_7);
  int current_tray_point = 0;

  while (true) {
    prev_slit = current_slit;
    current_slit = slit.read();
    limit_led_lower = limit_lower.read();
    limit_led_higher = limit_higher.read();
    if (tray_motor_polor > 0) {
      if (current_slit == LIGHT && prev_slit == DARK) {
        ++current_tray_point;
      }
    } else if (tray_motor_polor < 0) {
      if (current_slit == LIGHT && prev_slit == DARK) {
        --current_tray_point;
      }
    }
    /* if (limit_higher.read() == 1) { */
    /*   current_tray_point = 0; */
    /* } else if (limit_lower.read() == 1) { */
    /*   current_tray_point = 7; */
    /* } */

    switch (phase) {
    case -1:
      if (limit_lower.read() != 1) {
        tray_motor_polor = -1;
      } else {
        tray_motor_polor = 0;
      }
    case 0:
      if (current_tray_point < goal_tray_point) {
        tray_motor_polor = 1;
      } else if (current_tray_point > goal_tray_point) {
        tray_motor_polor = -1;
      } else {
        tray_motor_polor = 0;
      }
      break;
    }
    spinMotor(TRAY_MOTOR_ID, tray_motor_polor * MAX_TRAY_MOTOR_SPEED);
    current_point = current_tray_point;
  }
}
