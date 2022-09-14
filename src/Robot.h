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

  static Motor ShooterOne;
  static Motor ShooterTwo;
  static Motor RollerSpinner;
  static Motor Intake;
  static pros::GPS GPSSensor;
  static std::atomic<double> x;
  static IMU InertialSensor;

  static double turn_offset_y;
  static double turn_offset_x;


  static PD power_PD;
  static PD strafe_PD;
  static PD turn_PD;

  static ADIDigitalOut pneumatics;


  static std::map<std::string, std::unique_ptr<pros::Task>> tasks;

  static void odometry(void *ptr);
  static void move_to(vector<double> position, vector<double> margins= {1,1,1}, vector<double> speeds = {1, 1, 1}, bool pure_pursuit=false);


  static void start_task(string s, void(*func)(void *));
  static void end_task(string s);
  static bool task_exists(string s);

  static void display(void *ptr);
  static void drive(int power, int turn, int strafe);
  static void driveControl(void *ptr);
  static void autonomous();

};

#endif
