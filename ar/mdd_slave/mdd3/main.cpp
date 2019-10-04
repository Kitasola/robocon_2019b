#include <mbed.h>
#include <rotary_inc.hpp>
#include <scrp_slave.hpp>

ScrpSlave slave(PA_9, PA_10, PA_12, SERIAL_TX, SERIAL_RX, 0x0803e000);

constexpr int NUM_PORT = 5;

constexpr int NUM_MOTOR_PORT = 4;
constexpr int MAX_PWM = 250;
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

DigitalOut arm_solenoid[2] = {DigitalOut(PB_6), DigitalOut(PA_11)};
DigitalOut arm_led[2] = {DigitalOut(PB_3), DigitalOut(PB_4)};
bool solenoid(int cmd, int rx_data, int &tx_data) {
  arm_solenoid[0].write(rx_data % 10);
  arm_led[0].write(rx_data % 10);
  arm_solenoid[1].write((rx_data / 10) % 10);
  arm_led[1].write((rx_data / 10) % 10);
  return true;
}

int three_hight_current[2] = {};
bool checkthreeRegister(int cmd, int rx_data, int &tx_data) {
  if (rx_data >= 10) {
    tx_data = three_hight_current[rx_data - 10] / 10;
  } else {
    tx_data = three_hight_current[rx_data] % 10;
  }
  return true;
}

int three_stage_speed[2] = {};
bool checkthreeVelocity(int cmd, int rx_data, int &tx_data) {
  tx_data = three_stage_speed[rx_data];
  return true;
}

bool safe(int cmd, int rx_data, int &tx_data) {
  for (int i = 0; i < 4; ++i) {
    spinMotor(i, 0);
  }
  return true;
}

int main() {
  slave.addCMD(255, safe);

  slave.addCMD(30, setThreeHigh);
  slave.addCMD(40, solenoid);
  while (true) {
    AnalogIn three_register AnalogIn(PB_0);
    constexpr float THREE_REGISTER_MULTI = 10; // mmへの変換倍率
    constexpr int THREE_STAGE_OFFSET = 0;
    int measure_high[NUM_REGISTER] = {};
    for (int 0; i < NUM_REGISTER; ++i) {
      measure_high[i] =
          hight_register[i].read() * REGISTER_MULTI - TOW_STAGE_OFFSET;
    }
  }
}
