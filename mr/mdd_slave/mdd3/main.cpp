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

constexpr int TARY_ROCK_ANGLE = 1, TARY_FREE_ANGLE = 0;
constexpr double WAIT_TRAY_SERVO = 1.5;
void rockTray(int level) {
  DigitalOut rock_tray_servo(PA_1);
  rock_tray_servo = level;
}

constexpr int HAND_CATCH_ANGLE = 1, HAND_RELEASE_ANGLE = 0;
constexpr double WAIT_HAND_SERVO = 1;
void actHand(int level) {
  DigitalOut hand_servo(PA_3);
  hand_servo = level;
}

Timer time;
int phase = 7;
int goal_stroke = 0;
int goal_shoot_stroke = 0;
bool startShoot(int cmd, int rx_data, int &tx_data) {
  goal_shoot_stroke = rx_data;
  time.reset();
  phase = 3;
  return true;
}

int current_stroke = 0;
bool setStroke(int cmd, int rx_data, int &tx_data) {
  goal_stroke = rx_data;
  tx_data = current_stroke;
  return true;
}

int goal_shoot_tray_speed = 0, goal_tray_speed = 0;
bool setTraySpeed(int cmd, int rx_data, int &tx_data) {
  goal_shoot_tray_speed = rx_data;
  return true;
}

int check_stroke;
bool checkStroke(int cmd, int rx_data, int &tx_data) {
  tx_data = check_stroke;
  return true;
}

int GLOBAL_STROKE_LOAD_LENGTH;
bool loadTray(int cmd, int rx_data, int &tx_data) {
  if (rx_data == 1) {
    phase = 0;
    goal_stroke = GLOBAL_STROKE_LOAD_LENGTH;
  } else if (rx_data == -1) {
    phase = 7;
  }
  return true;
}

bool actServo(int cmd, int rx_data, int &tx_data) {
  actHand(rx_data % 10);
  rockTray(rx_data / 10);
  return true;
}

int main() {
  time.start();
  slave.addCMD(30, startShoot);
  slave.addCMD(31, setTraySpeed);
  slave.addCMD(32, setStroke);
  slave.addCMD(34, loadTray);
  slave.addCMD(35, checkStroke);
  slave.addCMD(36, actServo);
  slave.addCMD(255, safe);

  constexpr int TRAY_MOTOR_ID = 2, TRAY_ENCODER_ID = 3;
  /* RotaryInc tray_rotary(ENCODER_PIN[TRAY_ENCODER_ID][0], */
  /*                       ENCODER_PIN[TRAY_ENCODER_ID][1], 512, 1); */
  /* PidPosition tray_speed(1.0, 0, 0, 0); */
  int current_tray_speed = 0;

  constexpr int STROKE_MOTOR_ID = 0, STROKE_ENCODER_ID = 0;
  RotaryInc stroke_rotary(ENCODER_PIN[STROKE_ENCODER_ID][0],
                          ENCODER_PIN[STROKE_ENCODER_ID][1], 256, 1);
  constexpr int MAX_STROKE_LENGTH = 370, MAX_STROKE_ERROR = 2;
  constexpr int STROKE_LOAD_LENGTH = 350, STROKE_READY_LENGTH = 200;
  GLOBAL_STROKE_LOAD_LENGTH = STROKE_LOAD_LENGTH;
  /* int current_stroke = 0, stroke_offset = -MAX_STROKE_LENGTH; */
  int stroke_offset = -MAX_STROKE_LENGTH;
  constexpr double STROKE_DIAMETER = -42;
  PidPosition stroke(6.0, 0, 0, 0);
  AnalogIn stroke_reset(PA_5);
  constexpr double WAIT_RELOAD_ROCK = 2, WAIT_RELOAD_CHARGE = 0.2,
                   WAIT_ROLL_TRAY = 2;
  constexpr int RELOAD_ROCK_SPEED = 10, RELOAD_CHARGE_SPEED = 100;
  int reload_speed = 0;
  bool reload_mode = false;
  DigitalOut shoot_rock(PA_6);
  goal_stroke = MAX_STROKE_LENGTH;
  phase = 7;

  while (true) {
    spinMotor(TRAY_MOTOR_ID, goal_tray_speed);
    current_stroke =
        stroke_rotary.get() / 256.0 * STROKE_DIAMETER * M_PI - stroke_offset;

    if (!reload_mode) {
      spinMotor(STROKE_MOTOR_ID, stroke.control(goal_stroke, current_stroke));
    } else {
      if (stroke_reset.read() > 0.3) {
        stroke_offset += current_stroke - MAX_STROKE_LENGTH;
        reload_speed = 0;
      }
      spinMotor(STROKE_MOTOR_ID, reload_speed);
    }
    check_stroke = current_stroke;

    switch (phase) {
    case 0: {
      if (abs(goal_stroke - current_stroke) < MAX_STROKE_ERROR) {
        actHand(HAND_RELEASE_ANGLE);
        time.reset();
        phase = 1;
      }
      break;
    }
    case 1: {
      if (time.read() > WAIT_HAND_SERVO) {
        rockTray(TARY_ROCK_ANGLE);
        time.reset();
        phase = 2;
      }
      break;
    }
    case 2: {
      if (time.read() > WAIT_TRAY_SERVO) {
        time.reset();
        goal_stroke = STROKE_LOAD_LENGTH;
      }
      break;
    }
    case 3: {
      goal_tray_speed = goal_shoot_tray_speed;
      if (time.read() > WAIT_ROLL_TRAY) {
        goal_stroke = goal_shoot_stroke;
        if (abs(goal_stroke - current_stroke) < MAX_STROKE_ERROR) {
          shoot_rock.write(1);
          time.reset();
          phase = 4;
        }
      }
      break;
    }
    case 4: {
      if (time.read() > WAIT_RELOAD_ROCK) {
        reload_mode = true;
        reload_speed = RELOAD_ROCK_SPEED;
        goal_tray_speed = 0;
        time.reset();
        phase = 5;
      }
      break;
    }
    case 5: {
      if (time.read() > WAIT_RELOAD_CHARGE) {
        reload_speed = RELOAD_CHARGE_SPEED;
        shoot_rock.write(0);
        phase = 6;
      }
      break;
    }
    case 6: {
      if (reload_speed == 0) {
        reload_mode = false;
        goal_stroke = STROKE_LOAD_LENGTH;
      }
      break;
    }
    case 7: {
      if (abs(goal_stroke - current_stroke) < MAX_STROKE_ERROR) {
        rockTray(TARY_FREE_ANGLE);
        time.reset();
        phase = 8;
      }
      break;
    }
    case 8: {
      if (time.read() > WAIT_TRAY_SERVO) {
        actHand(HAND_CATCH_ANGLE);
        time.reset();
        phase = 9;
      }
      break;
    }
    case 9: {
      if (time.read() > WAIT_HAND_SERVO) {
        goal_stroke = STROKE_LOAD_LENGTH;
        time.reset();
      }
      break;
    }
    }
    wait(0.01);
  }
}
