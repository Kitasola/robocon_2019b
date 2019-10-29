//立ルンです, Ma/nLoop from 294
#include "raspi_utility/include/dualshock3.hpp"
#include "raspi_utility/include/gy521.hpp"
#include "raspi_utility/include/motor_serial.hpp"
#include "raspi_utility/include/pid.hpp"
#include "raspi_utility/include/pigpiod.hpp"
#include "raspi_utility/include/time.hpp"
#include <cmath>
#include <iostream>
#include <pigpio.h>
#include <string>
#include <vector>

using namespace std;
using namespace arrc_raspi;

double map(double value, double from_low, double from_high, double to_low,
           double to_high) {
  if (value > from_high) {
    value = from_high;
  } else if (value < from_low) {
    value = from_low;
  }
  return (value - from_low) * (to_high - to_low) / (from_high - from_low) +
         to_low;
}

bool checkError(double goal, double now, double error) {
  if (goal - error < now && now < goal - error) {
    return true;
  } else {
    return false;
  }
}

double calcTriangleTheta(double a, double b, double c) {
  return acos((a * a + b * b - c * c) / (2 * a * b));
}

int main() {
  // Serial
  MotorSerial ms;

  // DualShock3
  DualShock3 controller;

  // Check LED
  constexpr int RUN_LED = 13;
  pigpio.set(RUN_LED, OUT, 0);

  // Calibration
  UPDATELOOP(controller,
             !(controller.button(RIGHT) && controller.button(SQUARE))) {}
  constexpr double START_YAW = 0;
  Gy521 gyro(0x68, 2, 1000, 1.01);

  // Wheel ver three omuni
  constexpr int NUM_WHEEL = 3, WHEEL_MDD_ID[NUM_WHEEL] = {1, 2, 3},
                WHEEL_CMD[NUM_WHEEL] = {2, 2, 2};
  constexpr double WHEEL_OFFSET_THETA[NUM_WHEEL] = {};
  constexpr int MAX_ROBOT_SPEED = 200, MAX_ROBOT_MOMENT = 100,
                MAX_WHEEL_SPEED = 250;
  PidPosition robot_pose(0.0, 0.0, 0.0, MAX_ROBOT_MOMENT);

  // Laundry
  constexpr int LAUNDRY_MDD_ID = 3, LAUNDRY_CMD = 20;
  constexpr int LAUNDRY_ARM_X = 360, LAUNDRY_ARM_Y = 860;

  // Arm
  constexpr int ARM_MDD_ID = 6, SHOULDER_CMD = 29, ELBO_CMD = 20;
  constexpr int FRONT_ARM_LENGTH = 400 + 100, SECOND_ARM_LENGTH = 500;
  constexpr int ARM_START_X = 0, ARM_START_Y = 0;
  constexpr double OFFSET_SHOULDER_ANGLE = 0, OFFSET_ELBO_ANGLE = 10;
  constexpr double ARM_STICK_SPEED = 0.1;

  Timer timer;
  cout << "Main Start" << endl;
  // MainLoop
  UPDATELOOP(controller,
             !(controller.button(START) && controller.button(CROSS))) {
    // Sensor Update
    gyro.update();
    timer.update();
    timer.reset();

    // Wheel Goal Input Manual
    double stick_x = controller.stick(LEFT_X) / 128;
    double stick_y = -controller.stick(LEFT_Y) / 128;
    double robot_theta = atan2(stick_y, stick_x) - gyro.yaw / 180 * M_PI;
    double robot_speed = MAX_ROBOT_SPEED * hypot(stick_x, stick_y) *
                         (0.3 * abs(cos(2 * robot_theta) + 0.7));
    if (!controller.button(L1)) {
      robot_speed *= 0.5;
    }
    static double goal_yaw = START_YAW;
    if (controller.press(LEFT)) {
      goal_yaw = 90;
    } else if (controller.press(RIGHT)) {
      goal_yaw = -90;
    }

    // Arm Goal Input Manual
    static double arm_goal_x = ARM_START_X;
    static double arm_goal_y = ARM_START_Y;
    double arm_diff_x = ARM_STICK_SPEED * controller.stick(RIGHT_X) / 128;
    double arm_diff_y = ARM_STICK_SPEED * -controller.stick(RIGHT_Y) / 128;
    arm_goal_x += arm_diff_x * sin(gyro.yaw / 180 * M_PI) -
                  arm_diff_y * cos(gyro.yaw / 180 * M_PI);
    arm_goal_y += arm_diff_x * cos(gyro.yaw / 180 * M_PI) +
                  arm_diff_y * sin(gyro.yaw / 180 * M_PI);

    // Shoot
    static int phase = 0;
    switch (phase) {
    case 0:
      if (controller.press(SQUARE)) {
        phase = 1;
      }
      break;
    case 1:

      break;
    }

    // Laundry
    static int laundry_mode = 0;
    if (controller.button(CROSS) && laundry_mode == 2) {
      laundry_mode = 0;
      ms.send(LAUNDRY_MDD_ID, LAUNDRY_CMD, laundry_mode);
    } else if (controller.press(CROSS) && laundry_mode == 0) {
      laundry_mode = 2;
      ms.send(LAUNDRY_MDD_ID, LAUNDRY_CMD, laundry_mode);
    }
    // 排出のための腕回避
    if (laundry_mode == 0) {
      arm_goal_x = LAUNDRY_ARM_X;
      arm_goal_y = LAUNDRY_ARM_Y;
    }

    // Wheel Output
    double diff_yaw = goal_yaw - gyro.yaw;
    diff_yaw = diff_yaw - (int)diff_yaw / 180 * 360;
    double moment = robot_pose.control(diff_yaw);

    double wheel_goal_speed[NUM_WHEEL];
    double dummy_max = MAX_WHEEL_SPEED;
    for (int i = 0; i < NUM_WHEEL; ++i) {
      wheel_goal_speed[i] =
          robot_speed * cos(robot_theta + WHEEL_OFFSET_THETA[i]) + moment;
      if (wheel_goal_speed[i] > dummy_max) {
        dummy_max = wheel_goal_speed[i];
      }
    }
    for (int i = 0; i < NUM_WHEEL; ++i) {
      wheel_goal_speed[i] *= dummy_max / MAX_WHEEL_SPEED;
      ms.send(WHEEL_MDD_ID[i], WHEEL_CMD[i], wheel_goal_speed[i]);
    }

    // Arm Output
    double arm_radius = hypot(arm_goal_x, arm_goal_y);
    double angle_shoulder =
        calcTriangleTheta(SECOND_ARM_LENGTH, arm_radius, FRONT_ARM_LENGTH) +
        atan2(arm_goal_y, arm_goal_x);
    double angle_elbo =
        calcTriangleTheta(FRONT_ARM_LENGTH, SECOND_ARM_LENGTH, arm_radius);
    ms.send(ARM_MDD_ID, SHOULDER_CMD,
            angle_shoulder / M_PI * 180 + OFFSET_SHOULDER_ANGLE);
    ms.send(ARM_MDD_ID, ELBO_CMD, angle_elbo / M_PI * 180 + OFFSET_ELBO_ANGLE);
  }
finish:
  cout << "Main Finish" << endl;
  ms.send(255, 255, 0);
}
