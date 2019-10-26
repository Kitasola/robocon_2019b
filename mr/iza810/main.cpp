//立ルンです, Ma/nLoop from 294
#include "raspi_utility/include/pigpiod.hpp"
#include "raspi_utility/include/motor_serial.hpp"
#include "raspi_utility/include/gy521.hpp"
#include "raspi_utility/include/dualshock3.hpp"
#include <cmath>
#include <iostream>
#include <pigpio.h>
#include <string>
#include <time.h>
#include <unistd.h>
#include <vector>
#define ROOT3 (1.7320508)
#define M_PI_3 (M_PI / 3)
#define M_PI_6 (M_PI / 6)

using namespace std;
using namespace arrc_raspi;

struct pointinfo {
  int x;
  int y;
  int yaw;
  bool shoot;
  int table;
  int ultra;
};

inline double wheel_Func(double rad);
inline double check(double a, double b, double c);
inline bool yaw_check(int goal, int now);
constexpr double ErrorYaw = 3;

int main(void) {
  MotorSerial ms;
  // MDD通信セットアップ
  try {
    ms.init();
  } catch (const char *str) {
    return -1;
  }

  //----------etc.----------
  int restart = 0;
  // Check LED
  constexpr int BCheck = 13;
  gpioSetMode(BCheck, PI_OUTPUT);

  constexpr int CheckRed = 26, CheckBlue = 8;
  gpioSetMode(CheckRed, PI_INPUT);
  gpioSetPullUpDown(CheckRed, PI_PUD_DOWN);
  gpioSetMode(CheckBlue, PI_INPUT);
  gpioSetPullUpDown(CheckBlue, PI_PUD_DOWN);

  constexpr int ZoneRed = 20, ZoneBlue = 5;
  int flagZone = 0;
  gpioSetMode(ZoneRed, PI_OUTPUT);
  gpioWrite(ZoneRed, 0);
  gpioSetMode(ZoneBlue, PI_OUTPUT);
  gpioWrite(ZoneBlue, 0);

  constexpr int LEDCal = 7, LEDReset = 6;
  gpioSetMode(LEDCal, PI_OUTPUT);
  gpioWrite(LEDCal, 0);
  gpioSetMode(LEDReset, PI_OUTPUT);
  gpioWrite(LEDReset, 0);

  constexpr int CheckCal = 19, CheckReset = 16;
  gpioSetMode(CheckCal, PI_INPUT);
  gpioSetPullUpDown(CheckCal, PI_PUD_DOWN);
  gpioSetMode(CheckReset, PI_INPUT);
  gpioSetPullUpDown(CheckReset, PI_PUD_DOWN);

  int phase = 0;

  //---------Time----------
  struct timespec now, prev;
  long double delta, start = 0;

  //----------Shoot---------
  constexpr int ShootR = 291, ShootL = 293;
  constexpr int ShootRID = 5, ShootLID = 6;
  int shineR = 31, shineL = 29;
  bool flagShootR = false;
  ms.send(1, 31, ShootR);
  ms.send(1, 31, ShootL);
  ms.send(1, 32, 400);
  ms.send(1, 32, 400);

  //----------Plan Root----------
  vector<struct pointinfo> PointTable;
  int pointCount = 0;
  int setCount = 0;
  struct pointinfo goal;

  // TwoTable
  constexpr int TwoTableX = 3000, TwoTableY = 3500, TwoTableR = 1295;
  // constexpr int TwoTableDiv = 12;

  // MoveTable
  constexpr int MoveTableY[3] = {5500, 6500, 7500};
  int MoveTableX[3] = {2250, 2250, 2250};
  constexpr int MoveTableR[3] = {1290, 1290, 1290};

  // bia Field View, also yawGoal = moment
  double lengthF, angleF, velocityF;

  //----------Movement----------
  // OutPut
  constexpr int WheelID[3] = {1, 2, 3};
  constexpr int SpeedMax = 4600;
  constexpr int SpeedMin = 200;
  constexpr double SpeedLate = 240.0 / 4600;
  constexpr int MomentMax = 25;
  constexpr double WheelDeg[3] = {0, M_PI_3 * 2, -M_PI_3 * 2};
  double wheelSlow;
  double wheelGoal[3] = {};
  constexpr int ErrorMin = 10, ErrorSpeed = 350;

  // Input Robot View
  double velocityR = 0, angleR, moment;

  // Option
  // WheelSpeed Control from  Accel
  // Result: velocityOut[PWM], Goal: velocityGoal[PWM],
  // Control:velocityOut[PWM]
  constexpr double AccelMax = 2100;
  double velocityGoal[2] = {}, velocityAccel[2] = {}, velocityOut[2] = {},
         velocityDelta[2] = {};
  bool velocityDone[2] = {};
  long double accelTime[2][3] = {{}, {}}, accelStart = 0;
  int accelPolar[2] = {};
  long double stop = 0;
  constexpr double StopTime = 1.0;

  // Lock Angle bia PID
  // Result: yaw[degree], Goal: yawLock[degree], Control:
  // moment(define
  // before)[PWM]
  double yaw, yawDelta, yawPrev;
  double yawGoal;
  constexpr double yawProp = 7.2, yawInt = 51.42, yawDeff = 0.252;
  // constexpr double yawProp = 10, yawInt = 185.4, yawDeff = 0.672;

  // wheelSpeed control from PID
  int wheelCount[3] = {10, 10, 10}, wheelSpeedIn[3] = {},
      wheelSpeedInPrev[3] = {};
  long double wheelSpeedTimePrev[3] = {};
  double wheelSpeed[3] = {}, wheelDelta[3] = {}, wheelPrev[3] = {};
  int wheelOut[3] = {};
  constexpr double WheelProp[3] = {0.0021, 0.0021, 0.0021},
                   WheelInt[3] = {0, 0, 0}, WheelDeff[3] = {0, 0, 0};

  gpioWrite(BCheck, 1);
  //----------Calibration----------
  while (1) {
    if (gpioRead(CheckCal)) {
      gpioWrite(LEDCal, 1);
      if (gpioRead(CheckRed) || gpioRead(CheckBlue)) {
        break;
      }
    }
  }
  gpioWrite(LEDCal, 0);
  sleep(1);
  //----------Gyro----------
  GY521 gyro(0x68, 2, 1000, 1.01);

  //----------IncRotary----------
  constexpr int Range = 256 * 2;
  constexpr double WheelCirc = 100.6 * M_PI;
  rotaryInc rotary[3] = {rotaryInc(27, 17, true), rotaryInc(11, 9, true),
                         rotaryInc(10, 22, true)};
  int wheelIn[3] = {};
  int wheelInPrev[3] = {};
  double wheelDiff[3] = {};

  //----------Zone Check----------
  gpioWrite(ZoneRed, 1);
  gpioWrite(ZoneBlue, 1);
  while (1) {
    if (gpioRead(CheckRed)) {
      flagZone = 1;
      cout << "Red" << endl;
      --shineR;
      --shineL;
      break;
    } else if (gpioRead(CheckBlue)) {
      flagZone = -1;
      cout << "Blue" << endl;
      break;
    }
  }
  gpioWrite(ZoneRed, 0);
  gpioWrite(ZoneBlue, 0);

  //----------Guess Point----------
  // Origin Point = Centerof Robot Square
  constexpr int firstX = 490, firstY = 1530 + 89;
  constexpr double firstDeg = -90;
  double nowPoint[3] = {(double)(firstX * flagZone), firstY, firstDeg};
  double deltaX, deltaY, deltaL, deltaA;
  constexpr double MatrixPoint[3][3] = {{-2.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0},
                                        {0, 1.0 / ROOT3, -1.0 / ROOT3},
                                        {-1.0 / 3.0, -1.0 / 3.0, -1.0 / 3.0}};

  //----------Plan Root----------
  struct pointinfo dummyPoint;
  // dummyPoint = {500 * flagZone, TwoTableY, (int)firstDeg, false, 0, 0};
  // PointTable.push_back(dummyPoint);

  dummyPoint = {
      (TwoTableX - TwoTableR) * flagZone, TwoTableY, (int)firstDeg, true, 0, 0};
  PointTable.push_back(dummyPoint);

  dummyPoint = {500 * flagZone, MoveTableY[0], (int)firstDeg, false, 0, 0};
  PointTable.push_back(dummyPoint);

  dummyPoint = {(MoveTableX[0] - MoveTableR[0]) * flagZone,
                MoveTableY[0],
                (int)firstDeg,
                true,
                0,
                0};
  PointTable.push_back(dummyPoint);

  // dummyPoint = {500 * flagZone, MoveTableY[0], (int)firstDeg, false, 0, 0};
  // PointTable.push_back(dummyPoint);

  dummyPoint = {500 * flagZone, MoveTableY[1], (int)firstDeg, false, 0, 0};
  PointTable.push_back(dummyPoint);

  dummyPoint = {(MoveTableX[1] - MoveTableR[1]) * flagZone,
                MoveTableY[1],
                (int)firstDeg,
                true,
                0,
                0};
  PointTable.push_back(dummyPoint);

  // dummyPoint = {500 * flagZone, MoveTableY[1], (int)firstDeg, false, 0, 0};
  // PointTable.push_back(dummyPoint);

  dummyPoint = {500 * flagZone, MoveTableY[2], (int)firstDeg, false, 0, 0};
  PointTable.push_back(dummyPoint);

  dummyPoint = {(MoveTableX[2] - MoveTableR[2]) * flagZone,
                MoveTableY[2],
                (int)firstDeg,
                true,
                0,
                0};
  PointTable.push_back(dummyPoint);

  dummyPoint = {500 * flagZone, MoveTableY[2], (int)firstDeg, false, 0, 0};
  PointTable.push_back(dummyPoint);

  dummyPoint = {firstX * flagZone, firstY, (int)firstDeg, false, 0, 0};
  PointTable.push_back(dummyPoint);

  for (auto p : PointTable) {
    cout << p.x << ", " << p.y << ", " << p.yaw << endl;
  }

start:
  sleep(1);
  if (flagZone == 1) {
    gpioWrite(ZoneRed, 1);
    while (!gpioRead(CheckRed)) {
    }
  } else if (flagZone == -1) {
    gpioWrite(ZoneBlue, 1);
    while (!gpioRead(CheckBlue)) {
    }
  }

  cout << "Main Start" << endl;
  clock_gettime(CLOCK_REALTIME, &now);
  gyro.start(firstDeg * flagZone);
  phase = 0;

  // MainLoop
  while (1) {
    //----------Sensor----------
    // time
    prev = now;
    clock_gettime(CLOCK_REALTIME, &now);
    delta = now.tv_sec - prev.tv_sec +
            (long double)(now.tv_nsec - prev.tv_nsec) / 1000000000;
    start += delta;

    // GY521
    gyro.updata();
    yaw = gyro.yaw * flagZone;

    // RotaryInc
    for (int i = 0; i < 3; ++i) {
      wheelInPrev[i] = wheelIn[i];
      wheelIn[i] = rotary[i].get();
      wheelDiff[i] =
          (double)(wheelIn[i] - wheelInPrev[i]) * WheelCirc / (double)Range;
      if (wheelCount[i] == 10) {
        wheelSpeedInPrev[i] = wheelSpeedIn[i];
        wheelSpeedIn[i] = wheelIn[i];
        wheelSpeed[i] = (double)(wheelSpeedIn[i] - wheelSpeedInPrev[i]) *
                        WheelCirc / (double)Range /
                        (start - wheelSpeedTimePrev[i]);
        wheelSpeedTimePrev[i] = start;
        wheelCount[i] = 0;
      } else {
        ++wheelCount[i];
      }
    }

    //-----------Guess Field----------
    deltaX = deltaY = 0;
    for (int i = 0; i < 3; ++i) {
      deltaX += MatrixPoint[0][i] * wheelDiff[i];
      deltaY += MatrixPoint[1][i] * wheelDiff[i];
    }
    deltaL = hypot(deltaX, deltaY);
    deltaA = atan2(deltaY, deltaX);
    nowPoint[0] -= deltaL * cos(deltaA + flagZone * yaw * M_PI / 180);
    nowPoint[1] -= deltaL * sin(deltaA + flagZone * yaw * M_PI / 180);

    //----------Reset----------
    if (!gpioRead(CheckCal) && gpioRead(CheckReset)) {
      goto finish;
    }

    if (gpioRead(CheckReset)) {
      gyro.resetYaw(firstDeg * flagZone);
      nowPoint[0] = firstX * flagZone;
      nowPoint[1] = firstY;
      phase = 0;
      pointCount = 0;
      cout << "Reset" << endl;
      ms.send(255, 255, 0);
      goto start;
    }

    switch (phase) {
    //----------Set Goal----------
    case 0: {
      if ((ms.send(ShootRID, 10, 0) == 2) && (ms.send(ShootLID, 10, 0) == 2)) {
        goal = PointTable.at(pointCount);
        yawGoal = goal.yaw;

        if ((int)PointTable.size() != pointCount + 1) {
          ++pointCount;
        }
        phase = 1;
        setCount = 0;
      }

      break;
    }

    //----------Plan Output----------
    case 1: {
      lengthF = hypot(goal.y - nowPoint[1], goal.x - nowPoint[0]);
      if ((-ErrorMin < lengthF && lengthF < ErrorMin) || setCount > 6) {
        phase = 3;
        break;
      } else {
        phase = 2;
        ++setCount;
      }
      angleF = atan2(goal.y - nowPoint[1], goal.x - nowPoint[0]);
      velocityF = sqrt(2 * AccelMax * lengthF / M_PI + pow(SpeedMin, 2));
      if (velocityF > SpeedMax) {
        velocityF = SpeedMax;
      }

      velocityGoal[0] = velocityF * cos(angleF);
      velocityGoal[1] = velocityF * sin(angleF);
      double velocitySmall[2] = {ErrorSpeed * cos(angleF),
                                 ErrorSpeed * sin(angleF)};
      double dummylength[2] = {lengthF * fabs(cos(angleF)),
                               lengthF * fabs(sin(angleF))};
      accelStart = start;
      for (int i = 0; i < 2; ++i) {
        velocityDelta[i] = fabs(velocityGoal[i] - velocitySmall[i]);
        velocityAccel[i] = 0;
        accelTime[i][0] = accelTime[i][2] =
            M_PI * velocityDelta[i] / 2 / AccelMax;
        accelTime[i][1] =
            (dummylength[i] -
             2 * (pow(velocityGoal[i], 2) - pow(velocitySmall[i], 2)) * M_PI /
                 4 / AccelMax) /
            velocityGoal[i];
        if (accelTime[i][1] < 0) {
          accelTime[i][1] = 0;
        }

        accelTime[i][1] += accelTime[i][0];
        accelTime[i][2] += accelTime[i][1];

        if (velocityGoal[i] > velocityOut[i]) {
          accelPolar[i] = 1;
        } else {
          accelPolar[i] = -1;
        }
        velocityOut[i] = velocitySmall[i];
        velocityDone[i] = false;
      }

      break;
    }
    case 2: {
      //----------Movement----------
      // Input
      // WheelSpeed Control from Accel
      double time = start - accelStart;
      for (int i = 0; i < 2; ++i) {
        if (time < accelTime[i][0]) {
          velocityAccel[i] =
              AccelMax * sin(2 * AccelMax / velocityDelta[i] * time) * delta;
          velocityOut[i] += accelPolar[i] * velocityAccel[i];
        } else if (time < accelTime[i][1]) {
        } else if (time <= accelTime[i][2]) {
          velocityAccel[i] =
              -AccelMax *
              sin(2 * AccelMax / velocityDelta[i] * (time - accelTime[i][1])) *
              delta;
          velocityOut[i] += accelPolar[i] * velocityAccel[i];
        } else if (time > accelTime[i][2]) {
          velocityDone[i] = true;
        }
      }
      if (velocityDone[0] && velocityDone[1]) {
        phase = 1;
      }
      velocityR = hypot(velocityOut[0], velocityOut[1]);
      angleR =
          atan2(velocityOut[1], velocityOut[0]) - flagZone * yaw * M_PI / 180;

      // moment from Lock Angle bia PID
      switch (goal.table) {
      case 1:
        yawGoal = atan2(TwoTableY - nowPoint[1],
                        (flagZone * TwoTableX - nowPoint[0])) *
                      180 / M_PI -
                  90;
        yawGoal *= flagZone;
        break;
      }
      yawPrev = yawDelta;
      yawDelta = yawGoal - yaw;
      if (yawDelta > 180) {
        yawDelta -= 360;
      } else if (yawDelta <= -180) {
        yawDelta += 360;
      }
      moment = yawProp * yawDelta + yawInt * yawDelta * delta +
               yawDeff * (yawDelta - yawPrev) / delta;
      moment *= -1 * flagZone;

      if (moment > MomentMax) {
        moment = MomentMax;
      } else if (moment < -MomentMax) {
        moment = -MomentMax;
      }
      moment /= SpeedLate;

      // WheelOut
      int dummyMax = SpeadMax;
      for (int i = 0; i < 3; ++i) {
        wheelGoal[i] = velocityR * wheel_Func(angleR + WheelDeg[i]) + moment;
        if (abs(wheelGoal[i]) > dummyMax) {
          dummyMax = abs(wheelGoal[i]);
        }
      }
      wheelSlow = SpeadMax / (double)dummyMax;

      for (int i = 0; i < 3; ++i) {
        wheelGoal[i] *= wheelSlow;
        if (0 < wheelGoal[i] && wheelGoal[i] < SpeedMin) {
          wheelGoal[i] = 0;
        } else if (0 > wheelGoal[i] && wheelGoal[i] > -SpeedMin) {
          wheelGoal[i] = 0;
        }
      }

      // wheelSpeed control from PID
      for (int i = 0; i < 3; ++i) {
        wheelPrev[i] = wheelDelta[i];
        wheelDelta[i] = wheelGoal[i] - wheelSpeed[i];
        wheelOut[i] += WheelProp[i] * wheelDelta[i] +
                       WheelInt[i] * wheelDelta[i] * delta +
                       WheelDeff[i] * (wheelDelta[i] - wheelPrev[i]) / delta;
      }

      // Output
      // Data
      cout << start << ", ";
      // cout << goal.x << ", " << goal.y << ", " << goal.yaw << ", ";
      // cout << velocityOut[0] << ", " << velocityOut[1];
      for (int i = 0; i < 3; ++i) {
        cout << (int)wheelGoal[i] << ", " << wheelSpeed[i] << ", ";
      }
      cout << nowPoint[0] << ", " << nowPoint[1] << ", " << yaw;
      /*
      for (int i = 0; i < 3; ++i) {
        cout << wheelIn[i] << ",";
      }
      */
      cout << endl;

      for (int i = 0; i < 3; ++i) {
        ms.send(WheelID[i], 2, wheelOut[i]);
      }

      break;
    }
    case 3: {
      // Shoot
      if (goal.shoot) {
        for (int i = 0; i < 3; ++i) {
          ms.send(WheelID[i], 2, 0);
        }
        if (ms.send(ShootLID, 10, 0) == 2 && !flagShootR) {
          ms.send(ShootRID, 10, ShootR);
          ms.send(1, 30, shineR);
          flagShootR = true;
          phase = 0;
        } else if (ms.send(ShootRID, 10, 0) == 2) {
          ms.send(ShootLID, 10, ShootL);
          ms.send(1, 30, shineL);
          flagShootR = false;
          phase = 0;
        }
      } else {
        phase = 0;
      }
      break;
    }
    }
  }
finish:
  cout << "Main Finish" << endl;
  ms.send(255, 255, 0);
  gpioWrite(BCheck, 0);
  gpioWrite(ZoneRed, 0);
  gpioWrite(ZoneBlue, 0);
  gpioWrite(LEDCal, 0);
  gpioWrite(LEDReset, 0);
  return restart;
}

inline double wheel_Func(double rad) {
  while (rad < 0) {
    rad += 2 * M_PI;
  }
  while (rad >= 2 * M_PI) {
    rad -= 2 * M_PI;
  }

  if (0 <= rad && rad < M_PI_6) {
    return 1;
  } else if (M_PI_6 <= rad && rad < 5 * M_PI_6) {
    return (M_PI_2 - rad) * 3 / M_PI;
  } else if (5 * M_PI_6 <= rad && rad < 7 * M_PI_6) {
    return -1;
  } else if (7 * M_PI_6 <= rad && rad < 11 * M_PI_6) {
    return (rad - 3 * M_PI_2) * 3 / M_PI;
  } else if (11 * M_PI_6 <= rad && rad < 12 * M_PI_6) {
    return 1;
  }
  return 0;
}

inline double check(double a, double b, double c) {
  return (-b + sqrt(pow(b, 2) + 4 * a * c)) / (2 * a);
}

inline bool yaw_check(int goal, int now) {
  int small = goal + 360 - ErrorYaw;
  int large = goal + 360 + ErrorYaw;
  now = (now + 360) % 360 + 360;
  if (now > small && now < large) {
    return 1;
  }
  return 0;
}
