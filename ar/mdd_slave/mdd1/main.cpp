#include <mbed.h>
#include <rotary_inc.hpp>
#include <scrp_slave.hpp>

ScrpSlave slave(PA_9, PA_10, PA_12, SERIAL_TX, SERIAL_RX, 0x0803e000);

constexpr int NUM_PORT = 5;

constexpr int NUM_MOTOR_PORT = 4;
constexpr int MAX_PWM = 250;
constexpr double MAX_PWM_MBED = 0.95;
constexpr float PERIOD = 1 / 4000.0;
constexpr PinName MOTOR_PIN[NUM_MOTOR_PORT][3] = {{PB_0, PB_1, PB_3},
                                                  {PA_1, PA_3, PB_4},
                                                  {PA_8, PA_7, PB_5},
                                                  {PB_6, PA_11, PB_7}};

constexpr int NUM_ENCODER_PORT = 4;
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

int hanger_goal_speed = 0, hanger_current_speed = 0;
bool serveHanger(int cmd, int rx_data, int &tx_data) {
  hanger_goal_speed = rx_data;
  tx_data = hanger_current_speed;
  return true;
}

bool safe(int cmd, int rx_data, int &tx_data) {
  hanger_goal_speed = 0;
  return true;
}

int main() {
  slave.addCMD(20, serveHanger);
  slave.addCMD(255, safe);

  constexpr int NUM_HANGER_SW = 2;
  constexpr int HANGER_SW_PORT[NUM_HANGER_SW] = {2, 3};
  DigitalIn hanger_sw[NUM_HANGER_SW] = {
      DigitalIn(MOTOR_PIN[HANGER_SW_PORT[0]][0]),
      DigitalIn(MOTOR_PIN[HANGER_SW_PORT[1]][0])}; // PullUp
  DigitalOut hanger_led[NUM_HANGER_SW]{
      DigitalOut(MOTOR_PIN[HANGER_SW_PORT[0]][2]),
      DigitalOut(MOTOR_PIN[HANGER_SW_PORT[1]][2])};
  for (int i = 0; i < NUM_HANGER_SW; ++i) {
    hanger_sw[i].mode(PullUp);
    hanger_led[i] = hanger_sw[i].read();
  }

  while (true) {
    hanger_current_speed = hanger_goal_speed;
    int hanger_limit[NUM_HANGER_SW];
    for (int i = 0; i < NUM_HANGER_SW; ++i) {
      hanger_led[i] = hanger_limit[i] = hanger_sw[i].read();
    }
    if ((hanger_current_speed > 0 && hanger_limit[0] == 0) ||
        (hanger_current_speed < 0 && hanger_limit[1] == 0)) {
      hanger_current_speed = 0;
    }
    spinMotor(1, -hanger_current_speed);
  }
}
