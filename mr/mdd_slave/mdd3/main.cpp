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

PwmOut rock_tray_servo(PA_3);
constexpr int TARY_ROCK_ANGLE = 40, TARY_FREE_ANGLE = 0;
void rockTray(int degree) {
  rock_tray_servo.pulsewidth(map(degree, 0, 180, 0.5e-3, 2.4e-3));
  return true;
}
bool rockTray(int cmd, int rx_data, int &tx_data) {
  rockTray(rx_data);
  return true;
}

PwmOut hand_servo(PA_3);
constexpr int HAND_CATCH_ANGLE = 0, HAND_RELEASE_ANGLE = 90;
void actHand(int degree) {
  hand_servo.pulsewidth(map(degree, 0, 180, 0.5e-3, 2.4e-3));
  return true;
}
bool actHand(int cmd, int rx_data, int &tx_data) {
  actHand(rx_data);
  return true;
}

int phase;
int goal_stroke = 0;
bool startShoot(int cmd, int rx_data, int &tx_data) {
  goal_stroke = rx_data;
  return true;
}

bool setStroke(int cmd, int rx_data, int &tx_data) {
  goal_stroke = rx_data;
  return true;
}

int goal_tray_speed = 0;
bool setTraySpeed(int cmd, int rx_data, int &tx_data) {
  goal_tray_speed = rx_data;
  return true;
}

bool loadTray(int cmd, int rx_data, int &tx_data) {
  if (rx_data == 1) {
  } else if (rx_data == -1) {
  }
  return true;
}

int main() {
  slave.addCMD(2, spinMotor);
  slave.addCMD(3, spinMotor);
  slave.addCMD(4, spinMotor);
  slave.addCMD(5, spinMotor);
  slave.addCMD(30, startShoot);
  slave.addCMD(31, setTraySpeed);
  slave.addCMD(32, setStroke);
  slave.addCMD(33, rockTray);
  slave.addCMD(34, loadTray);
  slave.addCMD(40, actHand);
  slave.addCMD(255, safe);

  constexpr int TRAY_MOTOR_ID = 0, TRAY_ENCODER_ID = 1;
  /* RotaryInc tray_rotary(ENCODER_PIN[TRAY_ENCODER_ID][0], */
  /*                       ENCODER_PIN[TRAY_ENCODER_ID][1], 512, 1); */
  /* PidPosition tray_speed(1.0, 0, 0, 0); */
  int current_tray_speed = 0;

  constexpr int STROKE_MOTOR_ID = 1, STROKE_ENCODER_ID = 2;
  RotaryInc stroke_rotary(ENCODER_PIN[STROKE_ENCODER_ID][0],
                          ENCODER_PIN[STROKE_ENCODER_ID][1], 512, 1);
  constexpr int MAX_STROKE_LENGTH = 350;
  int current_stroke = 0, stroke_offset = 0;
  constexpr double STROKE_DIAMETER = 100;
  PidPosition stroke(1.0, 0, 0, 0);
  DigitalIn stroke_reset(PA_1);
  stroke_reset.mode(PULL_UP);

  while (true) {
    spinMotor(TRAY_MOTOR_ID, goal_tray_speed);

    current_stroke =
        stroke_rotary.getSum() * STROKE_DIAMETER * M_PI - stroke_offset;
    if (stroke_reset.read() == 1) {
      stroke_offset = current_stroke - MAX_STROKE_LENGTH;
    }
    spinMotor(STROKE_MOTOR_ID, stroke.control(goal_stroke, current_stroke));
  }
}
