#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <mbed.h>
#include <pid.hpp>
#include <ros.h>
#include <rotary_inc.hpp>
#include <scrp_slave.hpp>
#include <std_msgs/Float32.h>
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
geometry_msgs::Pose2D robot_pose;
ros::Publisher robot_pose_pub("/wheel/robot_pose", &robot_pose);
geometry_msgs::Twist debug_velocity;
ros::Publisher debug_velocity_pub("/wheel/debug_velocity", &debug_velocity);
void getTwist(const geometry_msgs::Twist &msgs) { goal_twist = msgs; }
ros::Subscriber<geometry_msgs::Twist> velocity_sub("/wheel/velocity",
                                                   &getTwist);
void resetRoboPose(const geometry_msgs::Pose2D &msgs) { robot_pose = msgs; }
ros::Subscriber<geometry_msgs::Pose2D> robot_pose_sub("/wheel/reset_robot_pose",
                                                      &resetRoboPose);

int main() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(robot_pose_pub);
  nh.advertise(debug_velocity_pub);
  nh.subscribe(velocity_sub);
  nh.subscribe(robot_pose_sub);

  /* constexpr double MAIN_FREQUENCY = 1000; */
  constexpr double TOPIC_FREQUENCY = 50;

  constexpr double PWM_PERIOD = 50; // 20 kHz

  /* 駆動輪 */
  constexpr double INVERCE_ROOT_2 = 1 / sqrt(2);
  constexpr int NUM_AXIS = 3;
  constexpr double DRIVE_MATRIX[4][3] = {
      {INVERCE_ROOT_2, -INVERCE_ROOT_2, -1},
      {-INVERCE_ROOT_2, -INVERCE_ROOT_2, -1},
      {-INVERCE_ROOT_2, INVERCE_ROOT_2, -1},
      {INVERCE_ROOT_2, INVERCE_ROOT_2, -1},
  };
  PinName drive_motor[NUM_WHEEL][2] = {
      {PC_9, PC_8}, {PB_5, PB_4}, {PB_14, PB_13}, {PA_11, PB_1}};
  PinName drive_led[NUM_WHEEL] = {PA_10, PB_15, PC_6, PB_2};
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
  constexpr int DRIVE_ROTARY_RANGE = 256, DRIVE_ROTARY_MULTI = 1;
  constexpr double MIN_DRIVE_SPEED = 0;
  constexpr double DRIVE_WHEEL_DIAMETER = 101.6;
  RotaryInc drive_rotary[NUM_WHEEL] = {
      RotaryInc(PC_2, PC_3, DRIVE_WHEEL_DIAMETER * M_PI, DRIVE_ROTARY_RANGE,
                DRIVE_ROTARY_MULTI),
      RotaryInc(PA_14, PA_15, DRIVE_WHEEL_DIAMETER * M_PI, DRIVE_ROTARY_RANGE,
                DRIVE_ROTARY_MULTI),
      RotaryInc(PC_4, PA_13, DRIVE_WHEEL_DIAMETER * M_PI, DRIVE_ROTARY_RANGE,
                DRIVE_ROTARY_MULTI),
      RotaryInc(PC_10, PC_11, DRIVE_WHEEL_DIAMETER * M_PI, DRIVE_ROTARY_RANGE,
                DRIVE_ROTARY_MULTI)};
  PidPosition drive_speed[NUM_WHEEL] = {
      PidPosition(0.00030, 0.007, 0.0000007, 1.0),
      PidPosition(0.00030, 0.005, 0.0000005, 1.0),
      PidPosition(0.00028, 0.005, 0.0000007, 1.0),
      PidPosition(0.00029, 0.005, 0.0000007, 1.0)};
  double drive_velocity[NUM_WHEEL] = {}, drive_filter = 0;

  /* 計測輪 */
  constexpr int MEASURE_ROTARY_RANGE = 256, MEASURE_ROTARY_MULTI = 2;
  constexpr double MEASURE_WHEEL_DIAMETER = 50.8 * 0.99, MEASURE_RADIUS = 312;
  RotaryInc measure_rotary[NUM_WHEEL] = {
      RotaryInc(PC_0, PC_1, MEASURE_WHEEL_DIAMETER * M_PI, MEASURE_ROTARY_RANGE,
                MEASURE_ROTARY_MULTI),
      RotaryInc(PA_12, PC_5, MEASURE_WHEEL_DIAMETER * M_PI,
                MEASURE_ROTARY_RANGE, MEASURE_ROTARY_MULTI),
      RotaryInc(PA_8, PA_9, MEASURE_WHEEL_DIAMETER * M_PI, MEASURE_ROTARY_RANGE,
                MEASURE_ROTARY_MULTI),
      RotaryInc(PA_6, PA_7, MEASURE_WHEEL_DIAMETER * M_PI, MEASURE_ROTARY_RANGE,
                MEASURE_ROTARY_MULTI)};

  /* 余剰PWMピン */
  /* constexpr PinName OTHER_PWM_PIN[3][3] = { */
  /*     {PB_6, PB_7, PB_12}, {PB_8, PB_9, PC_7}, {PA_0, PA_1, PB_0}}; */

  DigitalOut run_led(LED1);
  run_led = 1;
  DigitalIn calibration_switch(PC_13); //青色のボタン

  /* ==========ここより上にしかパラメータは存在しません========== */
  Timer main_loop, topic_loop;
  main_loop.start();
  topic_loop.start();

  robot_pose.x = 0;
  robot_pose.y = 0;
  robot_pose.theta = M_PI;
  while (true) {
    nh.spinOnce();
    /* if (topic_loop.read() > 1.0 / MAIN_FREQUENCY) { */
    double delta_t = main_loop.read();
    main_loop.reset();

    run_led = !run_led;
    if (topic_loop.read() > 1.0 / TOPIC_FREQUENCY) {
      topic_loop.reset();
      robot_pose_pub.publish(&robot_pose);
      debug_velocity_pub.publish(&debug_velocity);
    }

    // Odometry
    double measure_diff[NUM_WHEEL] = {};
    for (int i = 0; i < NUM_WHEEL; ++i) {
      measure_diff[i] = measure_rotary[i].diff();
    }
    double robot_x = measure_diff[0] / 2 - measure_diff[2] / 2;
    double robot_y = -measure_diff[1] / 2 + measure_diff[3] / 2;
    double robot_theta = 0;
    for (int i = 0; i < NUM_WHEEL; ++i) {
      robot_theta += -measure_diff[i];
    }
    robot_theta /= MEASURE_RADIUS * NUM_WHEEL;
    robot_pose.theta += robot_theta;
    if (robot_pose.theta > M_PI) {
      robot_pose.theta -= 2 * M_PI;
    } else if (robot_pose.theta <= -M_PI) {
      robot_pose.theta += 2 * M_PI;
    }
    robot_pose.x +=
        robot_x * cos(robot_pose.theta) - robot_y * sin(robot_pose.theta);
    robot_pose.y +=
        robot_x * sin(robot_pose.theta) + robot_y * cos(robot_pose.theta);

    // Move
    // Exchange to Robot, from Field
    double robot_velocity[NUM_AXIS] = {
        goal_twist.linear.x * cos(robot_pose.theta) -
            goal_twist.linear.y * sin(robot_pose.theta),
        goal_twist.linear.x * sin(robot_pose.theta) +
            goal_twist.linear.y * cos(robot_pose.theta),
        goal_twist.angular.z};
    double drive_goal[NUM_WHEEL] = {};
    for (int i = 0; i < NUM_WHEEL; ++i) {
      double drive_control;
      drive_velocity[i] = drive_velocity[i] * drive_filter +
                          drive_rotary[i].getSpeed() * (1 - drive_filter);
      if ((int)goal_twist.angular.y == -1) {
        drive_speed[i].control(drive_goal[i], drive_velocity[i]);
        drive_speed[i].reset();
        drive_control = 0;
      } else {
        for (int j = 0; j < NUM_AXIS; ++j) {
          drive_goal[i] += DRIVE_MATRIX[i][j] * robot_velocity[j];
        }
        drive_control =
            drive_speed[i].control(drive_goal[i], drive_velocity[i]);
      }
      spinMotor(i, drive_control);
    }

    // Debug
    debug_velocity.angular.y = goal_twist.angular.y;
    switch ((int)debug_velocity.angular.y) {
      // Velocity
    case 1:
      debug_velocity.linear.x = drive_velocity[0];
      debug_velocity.linear.y = drive_velocity[1];
      debug_velocity.linear.z = drive_velocity[2];
      debug_velocity.angular.x = drive_velocity[3];
      break;
    default:
      debug_velocity = goal_twist;
      break;
    }
    /* } else { */
    /* wait_ms(1.0 / MAIN_FREQUENCY / 2.0); */
    /* } */
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