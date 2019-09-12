#include <mbed.h>
#include <pid.hpp>
#include <rotary_inc.hpp>
#include <scrp_slave.hpp>

using namespace arrc;

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

int goal_two_hight = 0;          // [mm]
int dummy_current_two_hight = 0; // [mm]
bool setTwoHigh(int cmd, int rx_data, int &tx_data) {
  // rx_data, tx_data [cm]
  goal_two_hight = rx_data * 10;
  tx_data = dummy_current_two_hight / 10;
  return true;
}

int two_max_velocity = 0; // mm/s
bool setTwoVelocity(int cmd, int rx_data, int &tx_data) {
  // rx_data, tx_data [cm]
  two_max_velocity = rx_data * 10;
  return true;
}

int two_max_accel = 0; // mm/s^2
bool setTwoAccel(int cmd, int rx_data, int &tx_data) {
  // rx_data, tx_data [cm]
  two_max_accel = rx_data * 10;
  return true;
}

int two_hight_current[2] = {};
bool checkTwoRegister(int cmd, int rx_data, int &tx_data) {
  tx_data = two_hight_current[rx_data] / 10;
  return true;
}

DigitalOut arm_solenoid[2] = {DigitalOut(PB_6), DigitalOut(PA_11)};
DigitalOut arm_led[2] = {DigitalOut(PB_3), DigitalOut(PB_4)};
bool solenoid(int cmd, int rx_data, int &tx_data) {
  arm_solenoid[0].write(rx_data % 10);
  arm_led[0].write(rx_data % 10);
  arm_solenoid[1].write((rx_data / 10) % 10);
  arm_led[1].write((rx_data / 10) % 10);
  return true;
}

int main() {
  Timer time;
  time.start();
  if (PORT_FUNCTION[0] == 0) {
    motor_pwm[0][0] = new PwmOut(MOTOR_PIN[0][0]);
    motor_pwm[0][0]->period(PERIOD);
    motor_pwm[0][1] = new PwmOut(MOTOR_PIN[0][1]);
    motor_pwm[0][1]->period(PERIOD);
    motor_led[0] = new DigitalOut(MOTOR_PIN[0][2]);
    slave.addCMD(2, spinMotor);
  }
  if (PORT_FUNCTION[1] == 1) {
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

  slave.addCMD(30, setTwoHigh);
  slave.addCMD(31, setTwoVelocity);
  slave.addCMD(32, setTwoAccel);
  slave.addCMD(33, checkTwoRegister);
  slave.addCMD(40, solenoid);
  while (true) {
    float delta_t = time.read();
    time.reset();
    constexpr int NUM_TWO_REGISTER = 2;
    AnalogIn two_register[NUM_TWO_REGISTER] = {AnalogIn(PB_0),
                                               AnalogIn(PA_0)}; // right, left
    constexpr float TWO_REGISTER_MULTI = 210.0 / 255 * 720; // mmへの変換倍率
    constexpr int TOW_STAGE_OFFSET = 0;
    static int two_hight_prev[NUM_TWO_REGISTER] = {};
    int two_velocity[NUM_TWO_REGISTER] = {};
    PidVelocity two_motor[NUM_TWO_REGISTER] = {PidVelocity(0, 0, 0, 0),
                                               PidVelocity(0, 0, 0, 0)};

    for (int i = 0; i < NUM_TWO_REGISTER; ++i) {
      two_hight_current[i] =
          two_register[i].read() * TWO_REGISTER_MULTI - TOW_STAGE_OFFSET;
      spinMotor(i + 2,
                two_motor[i].control(
                    two_max_velocity,
                    (two_hight_current[i] - two_hight_prev[i]) / delta_t));
      two_hight_prev[i] = two_hight_current[i];
    }
  }
}
