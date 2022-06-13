#ifndef ROBOT_H
#define ROBOT_H

#include "main.h"
#include <atomic>
#include <memory>
#include <map>
#include <vector>
#include <atomic>

using namespace pros;
using namespace std;

class Robot {
public:
  static Controller Controller1;
  static Motor LeftBackWheel;
  static Motor RightBackWheel;
  static Motor LeftFrontWheel;
  static Motor RightFrontWheel;

  static Motor FourBar;
  static Motor ChainBar;
  static Motor GoalIntake;
  static Motor TowerSpinner;
  static pros::GPS GPSSensor;
  static std::atomic<double> x;FILE *fp;
  fp = fopen ("filename.txt","w");
  if (fp!=NULL)
  {
    fprintf(fp,"Some String\n");
    fclose (fp);
  }
  static ADIDigitalOut pneumatics;


  static std::map<std::string, std::unique_ptr<pros::Task>> tasks;

  static void start_task(string s, void(*func)(void *));
  static void end_task(string s);
  static bool task_exists(string s);

  static void display(void *ptr);
  static void drive(int power, int turn);
  static void driveControl(void *ptr);
  static void

  static void autonomousCode();
};
#endif
