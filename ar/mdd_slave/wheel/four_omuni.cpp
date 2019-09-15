#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <gy521.hpp>
#include <math.h>
#include <mbed.h>
#include <pid.hpp>
#include <ros.h>
#include <rotary_inc.hpp>
#include <scrp_slave.hpp>
#include <tf.h>

using namespace arrc;

constexpr int NUM_WHEEL = 4;
/* constexpr double M_PI = 3.14159; */
constexpr float MAX_PWM_RATIO = 0.98;
constexpr float MIN_PWM_RATIO = 0;
inline bool spinMotor(int id, double pwm);
PwmOut *g_drive_motor[NUM_WHEEL][2];
DigitalOut *g_drive_led[NUM_WHEEL];

ros::NodeHandle nh;
geometry_msgs::Twist goal_twist;
void getTwist(const geometry_msgs::Twist &msgs) { goal_twist = msgs; }
geometry_msgs::Pose2D robot_pose;
ros::Publisher robot_pose_pub("/wheel/robot_pose", &robot_pose);
geometry_msgs::Twist debug_velocity;
ros::Publisher debug_velocity_pub("/wheel/debug_velocity", &debug_velocity);
ros::Subscriber<geometry_msgs::Twist> velocity_sub("/wheel/velocity",
                                                   &getTwist);

int main() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();

  constexpr double MAIN_FREQUENCY = 1000;
  constexpr double TOPIC_FREQUENCY = 300;

  constexpr double PWM_PERIOD = 2048;

  /* 駆動輪 */
  constexpr double INVERCE_ROOT_2 = 1 / sqrt(2);
  constexpr int NUM_AXIS = 3;
  constexpr double DRIVE_MATRIX[4][3] = {
      {INVERCE_ROOT_2, INVERCE_ROOT_2, 1},
      {INVERCE_ROOT_2, -INVERCE_ROOT_2, 1},
      {-INVERCE_ROOT_2, -INVERCE_ROOT_2, 1},
      {-INVERCE_ROOT_2, INVERCE_ROOT_2, 1},
  };
  PinName drive_motor[NUM_WHEEL][2] = {
      {PB_1, PA_11}, {PB_13, PB_14}, {PB_4, PB_5}, {PC_8, PC_9}};
  PinName drive_led[NUM_WHEEL] = {PC_6, PB_2, PB_15, PA_10};
  /* PwmOut drive_motor[NUM_WHEEL][2] = {{PwmOut(PB_1), PwmOut(PA_11)}, */
  /*                                     {PwmOut(PB_13), PwmOut(PB_14)}, */
  /*                                     {PwmOut(PB_4), PwmOut(PB_5)}, */
  /*                                     {PwmOut(PC_8), PwmOut(PC_9)}}; */
  /* DigitalOut drive_led[NUM_WHEEL] = {DigitalOut(PC_6), DigitalOut(PB_2), */
  /*                                    DigitalOut(PB_15), DigitalOut(PA_10)};
   */
  for (int i = 0; i < NUM_WHEEL; ++i) {
    g_drive_motor[i][0] = new PwmOut(drive_motor[i][0]);
    g_drive_motor[i][0]->period_us(PWM_PERIOD);
    g_drive_motor[i][1] = new PwmOut(drive_motor[i][1]);
    g_drive_motor[i][1]->period_us(PWM_PERIOD);
    g_drive_led[i] = new DigitalOut(drive_led[i]);
  }
  constexpr int DRIVE_ROTARY_RANGE = 512, DRIVE_ROTARY_MULTI = 2;
  constexpr double DRIVE_WHEEL_DIAMETER = 101.6; //, DRIVE_TURN_DADIUS = 100;
  RotaryInc drive_rotary[NUM_WHEEL] = {
      RotaryInc(PC_10, PC_11, DRIVE_ROTARY_RANGE, DRIVE_ROTARY_MULTI),
      RotaryInc(PC_4, PA_13, DRIVE_ROTARY_RANGE, DRIVE_ROTARY_MULTI),
      RotaryInc(PA_14, PA_15, DRIVE_ROTARY_RANGE, DRIVE_ROTARY_MULTI),
      RotaryInc(PC_2, PC_3, DRIVE_ROTARY_RANGE, DRIVE_ROTARY_MULTI)};
  PidVelocity drive_speed[NUM_WHEEL] = {
      PidVelocity(0.00014, 0, 0, 0), PidVelocity(0.00014, 0, 0, 0),
      PidVelocity(0.00014, 0, 0, 0), PidVelocity(0.00014, 0, 0, 0)};

  /* 計測輪 */
  constexpr int MEASURE_ROTARY_RANGE = 256, MEASURE_ROTARY_MULTI = 22;
  constexpr double MEASURE_WHEEL_DIAMETER =
      50.8 * 0.99; // , MEASURE_TURN_DADIUS = 100;
  // A, B逆にするとバグる
  RotaryInc measure_rotary[NUM_WHEEL] = {
      RotaryInc(PA_12, PC_5, MEASURE_ROTARY_RANGE, MEASURE_ROTARY_MULTI),
      RotaryInc(PC_0, PC_1, MEASURE_ROTARY_RANGE, MEASURE_ROTARY_MULTI),
      RotaryInc(PA_6, PA_7, MEASURE_ROTARY_RANGE, MEASURE_ROTARY_MULTI),
      RotaryInc(PA_9, PA_8, MEASURE_ROTARY_RANGE, MEASURE_ROTARY_MULTI)};

  I2C i2c(PB_3, PB_10);
  GY521 gyro(i2c, 2, 1000, 1.012);

  /* 余剰PWMピン */
  /* constexpr PinName OTHER_PWM_PIN[3][3] = { */
  /*     {PB_6, PB_7, PB_12}, {PB_8, PB_9, PC_7}, {PA_0, PA_1, PB_0}}; */

  /* 何のピンなんやろ */
  DigitalOut run_led(LED1);
  DigitalIn calibration_switch(PC_13); //青色のボタン
  run_led = 1;

  /* ==========ここより上にしかパラメータは存在しません========== */
  nh.advertise(robot_pose_pub);
  nh.advertise(debug_velocity_pub);
  nh.subscribe(velocity_sub);

  Timer main_loop, topic_loop;
  main_loop.start();
  topic_loop.start();

  robot_pose.x = 0;
  robot_pose.y = 0;
  while (true) {
    nh.spinOnce();
    if (topic_loop.read() > 1.0 / MAIN_FREQUENCY) {
      double delta_t = main_loop.read();
      main_loop.reset();

      if (topic_loop.read() > 1.0 / TOPIC_FREQUENCY) {
        run_led = !run_led;
        topic_loop.reset();
        robot_pose_pub.publish(&robot_pose);
        debug_velocity_pub.publish(&debug_velocity);
      }
      gyro.updata();
      double robot_yaw = gyro.yaw / 180 * M_PI;

      double robot_velocity[NUM_AXIS] = {
          goal_twist.linear.x * cos(robot_yaw) -
              goal_twist.linear.y * sin(robot_yaw),
          goal_twist.linear.x * sin(robot_yaw) -
              goal_twist.linear.y * cos(robot_yaw),
          goal_twist.angular.z};
      double drive_goal[NUM_WHEEL] = {};
      for (int i = 0; i < NUM_WHEEL; ++i) {
        for (int j = 0; j < NUM_AXIS; ++j) {
          drive_goal[i] += DRIVE_MATRIX[i][j] * robot_velocity[j];
        }
        spinMotor(i, drive_speed[i].control(drive_goal[i],
                                            -drive_rotary[i].getSpeed() *
                                                DRIVE_WHEEL_DIAMETER * M_PI));
      }
      debug_velocity = goal_twist;

      double robot_x =
          -measure_rotary[0].getDiff() * MEASURE_WHEEL_DIAMETER * M_PI / 2 +
          measure_rotary[2].getDiff() * MEASURE_WHEEL_DIAMETER * M_PI / 2;
      double robot_y =
          -(-measure_rotary[1].getDiff()) * MEASURE_WHEEL_DIAMETER * M_PI / 2 +
          measure_rotary[3].getDiff() * MEASURE_WHEEL_DIAMETER * M_PI / 2;
      double field_yaw = robot_yaw;
      robot_pose.x += robot_x * cos(field_yaw) - robot_y * sin(field_yaw);
      robot_pose.y += robot_x * sin(field_yaw) + robot_y * cos(field_yaw);
      robot_pose.theta = field_yaw;
    } else {
      wait_ms(1.0 / MAIN_FREQUENCY / 2.0);
      /* wait_ms(1000); */
    }
  }
}

inline bool spinMotor(int id, double pwm) {
  if (pwm > MAX_PWM_RATIO) {
    pwm = MAX_PWM_RATIO;
  } else if (pwm < -MAX_PWM_RATIO) {
    pwm = -MAX_PWM_RATIO;
  }

  if (pwm > MIN_PWM_RATIO) {
    g_drive_motor[id][0]->write(pwm);
    g_drive_motor[id][1]->write(0);
    g_drive_led[id]->write(1);
  } else if (pwm < -MIN_PWM_RATIO) {
    g_drive_motor[id][0]->write(0);
    g_drive_motor[id][1]->write(-pwm);
    g_drive_led[id]->write(1);
  } else {
    g_drive_motor[id][0]->write(0);
    g_drive_motor[id][1]->write(0);
    g_drive_led[id]->write(0);
  }
  return true;
}