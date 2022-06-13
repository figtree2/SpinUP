#include "Robot.h"
#include "main.h"

using namespace std;
using namespace pros;



Controller Robot::Controller1(E_CONTROLLER_MASTER);
Motor Robot::LeftBackWheel(19);
Motor Robot::RightBackWheel(10, true);
Motor Robot::LeftFrontWheel(7);
Motor Robot::RightFrontWheel(17, true);

Motor Robot::FourBar(14);
Motor Robot::ChainBar(9);
Motor Robot::GoalIntake(11);
Motor Robot::TowerSpinner(20);

pros::GPS Robot::GPSSensor(2);

std::atomic<double> Robot::x = 0;

ADIDigitalOut Robot::pneumatics('A');

std::map<std::string, std::unique_ptr<pros::Task>> Robot::tasks;


bool Robot::task_exists(string s) {
  return tasks.find(s) != tasks.end();
}

//starts a task and puts the task inside of the map
void Robot::start_task(std::string name, void (*func)(void *)) {
	if (!task_exists(name)) {
		tasks.insert(std::pair<std::string, std::unique_ptr<pros::Task>>(name, std::move(std::make_unique<pros::Task>(func, &x, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, ""))));
	}
}

//removes a task form the map and terminates
 void Robot::end_task(string s) {
	 if(task_exists(s)) {
		 tasks.erase(s);
	 }
 }


void Robot::display(void *ptr) {
  while(true) {
    double *a;
    double *b;
    GPSSensor.get_offset(a, b);
    lcd::print(3, "Offset x: %3f, y: %3f", *a, *b);
    lcd::print(2, "hello :D");
  }
}

void Robot::drive(int power, int turn)
{
  int max_left_speed = (power + turn > 127) ? 127 : (power + turn);
  Robot::LeftFrontWheel = max_left_speed;
  Robot::LeftBackWheel = max_left_speed;
  Robot::RightFrontWheel = power - turn;
  Robot::RightBackWheel = power - turn;
}


void Robot::driveControl(void *ptr)
{
  while(true)
  {
    int power = Robot::Controller1.get_analog(ANALOG_LEFT_Y);
    int turn = Robot::Controller1.get_analog(ANALOG_RIGHT_X);
    drive(power, turn);
    //FourBar

    //pneumatics
    if(Robot::Controller1.get_digital(DIGITAL_A)) {
      Robot::pneumatics.set_value(HIGH);
      delay(500);
      Robot::pneumatics.set_value(LOW);
    }
    if(Robot::Controller1.get_digital(DIGITAL_L1)) {
      Robot::FourBar = 127;
    }
    else if (Robot::Controller1.get_digital(DIGITAL_L2)) {
      Robot::FourBar = -127;
    }
    else {
      Robot::FourBar = 0;
      Robot::FourBar.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    }

    //ChainBar
    if(Robot::Controller1.get_digital(DIGITAL_R1)) {
      Robot::ChainBar = 127;
    }
    else if (Robot::Controller1.get_digital(DIGITAL_R2)) {
      Robot::ChainBar = -127;
    }
    else if(Robot::Controller1.get_digital(DIGITAL_X)) {
      Robot::ChainBar = 20;
    }
    else if(Robot::Controller1.get_digital(DIGITAL_Y)) {
      Robot::ChainBar = -20;
    }
    else {
      Robot::ChainBar = 0;
      Robot::ChainBar.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    }


    //Tower GoalIntake
    if(Robot::Controller1.get_digital(DIGITAL_DOWN)) {
      Robot::GoalIntake = 127;
    }
    else if(Robot::Controller1.get_digital(DIGITAL_UP)) {
      Robot::GoalIntake = -127;
    }
    else {
      Robot::GoalIntake = 0;
    }

    //Goal Spinner
    if(Robot::Controller1.get_digital(DIGITAL_RIGHT)) {
      Robot::TowerSpinner = 127;
    }
    else if(Robot::Controller1.get_digital(DIGITAL_LEFT)) {
      Robot::TowerSpinner = -127;
    }
    else {
      Robot::TowerSpinner = 0;
    }

  }
}
