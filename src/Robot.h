#ifndef ROBOT_H
#define ROBOT_H

#include "main.h"
#include <atomic>
#include <memory>
#include <map>
#include <vector>
#include <atomic>
#include "PD.h"

using namespace pros;
using namespace std;

class Robot {
public:
  static Controller Controller1;
  static Motor LeftBackWheel;
  static Motor RightBackWheel;
  static Motor LeftFrontWheel;
  static Motor RightFrontWheel;
  static Motor LeftMiddleWheel;
  static Motor RightMiddleWheel;
  static Motor Catapult;

  static Motor ShooterOne;
  static Motor ShooterTwo;
  static Motor RollerSpinner;
  static Motor Intake;
  static pros::GPS GPSSensor;
  static std::atomic<double> x;
  static std::atomic<double> y;
  static IMU InertialSensor;

  static atomic<double> turn_offset_y;
  static atomic<double> turn_offset_x;
  static double turn_offset;


  static PD power_PD;
  static PD strafe_PD;
  static PD turn_PD;

  static ADIDigitalOut pneumatics;
static ADIEncoder LeftEncoder;
static ADIEncoder RightEncoder;
  static ADIEncoder BackEncoder;

  static ADIDigitalIn limit_switch;


  static std::map<std::string, std::unique_ptr<pros::Task>> tasks;

  static void odometry(void *ptr);
  static void move_to(std::vector<double> pose, double stop_threshold = .1, bool pure_pursuit = false, int flipout_timer = 0, std::vector<double> speeds = {1, 1, 1});


  static void start_task(string s, void(*func)(void *));
  static void end_task(string s);
  static bool task_exists(string s);

  static void reset_PD();
  static void brake(string mode);

  static void display(void *ptr);
  static void driveOdom(int power, int turn, int strafe);
  static void drive(int power, int turn, int strafe);
  static void driveControl(void *ptr);
  static void autonomous();

  static void windUp(void *ptr);
  static void rel(void *ptr);

  static void spin(void *ptr);
  static void shoot();

};

#endif
