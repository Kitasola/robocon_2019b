#include <mbed.h>
#include <pid.hpp>
#include <rotary_inc.hpp>
#include <scrp_slave.hpp>

ScrpSlave slave(PA_9, PA_10, PA_12, SERIAL_TX, SERIAL_RX, 0x0803e000);

using namespace arrc;

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

constexpr int NUM_SOLENOIDS = 2;
void actSolenoid(int port, int level) {
  DigitalOut solenoid[NUM_SOLENOIDS] = {DigitalOut(MOTOR_PIN[port][0]),
                                        DigitalOut(MOTOR_PIN[port][1])};
  DigitalOut led(MOTOR_PIN[port][2]);
  for (int i = 0; i < NUM_SOLENOIDS; ++i) {
    solenoid[i] = level;
  }
  led = level;
}

bool actSolenoid(int cmd, int rx_data, int &tx_data) {
  actSolenoid(cmd - 40, rx_data);
  return true;
}

int goal_arm_hight = 0;          // [mm]
int dummy_current_arm_hight = 0; // [mm]
bool setArmHigh(int cmd, int rx_data, int &tx_data) {
  // rx_data, tx_data [cm]
  goal_arm_hight = rx_data * 10;
  tx_data = dummy_current_arm_hight / 10;
  return true;
}

int arm_hight_current = 0;
bool checkArmRegister(int cmd, int rx_data, int &tx_data) {
  if (rx_data >= 10) {
    tx_data = arm_hight_current / 10;
  } else {
    tx_data = arm_hight_current % 10;
  }
  return true;
}

int arm_stage_speed = 0;
bool checkArmVelocity(int cmd, int rx_data, int &tx_data) {
  tx_data = arm_stage_speed;
  return true;
}

bool safe(int cmd, int rx_data, int &tx_data) {
  actSolenoid(40, 0);
  actSolenoid(40, 1);
  return true;
}

int main() {
  slave.addCMD(255, safe);

  slave.addCMD(30, setArmHigh);
  slave.addCMD(40, actSolenoid);
  slave.addCMD(41, actSolenoid);

  AnalogIn arm_register(PA_0);
  constexpr int ARM_STAGE_ID = 0;
  constexpr float ARM_REGISTER_MULTI = 720 / (210.0 / 255) * 5 / 3.3;
  constexpr int ARM_STAGE_OFFSET = -1196; // 127};
  constexpr double ARM_STAGE_DOWN = 0.5;
  PidPosition arm_motor(5.0, 0, 0, 0);

  while (true) {
    arm_hight_current =
        arm_register.read() * ARM_REGISTER_MULTI - ARM_STAGE_OFFSET;
    arm_stage_speed = arm_motor.control(goal_arm_hight, arm_hight_current);
    if (goal_arm_hight < arm_hight_current) {
      arm_stage_speed *= ARM_STAGE_DOWN;
    }
    spinMotor(ARM_STAGE_ID, arm_stage_speed);
  }
}
