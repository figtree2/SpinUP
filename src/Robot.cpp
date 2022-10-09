#include "Robot.h"
#include "main.h"
#include <math.h>
#include <atomic>
#include <deque>
using namespace std;
using namespace pros;
#define TO_RAD(n) n * M_PI / 180;


Controller Robot::Controller1(E_CONTROLLER_MASTER);
Motor Robot::LeftBackWheel(10);
Motor Robot::RightBackWheel(19, true);
Motor Robot::LeftFrontWheel(14);
Motor Robot::RightFrontWheel(1, true);

Motor Robot::ShooterOne(12);
Motor Robot::ShooterTwo(11, true);
Motor Robot::RollerSpinner(8);
Motor Robot::Intake(18);

pros::GPS Robot::GPSSensor(2);

IMU Robot::InertialSensor(17);

ADIDigitalOut Robot::pneumatics('H');
ADIEncoder Robot::RightEncoder('A', 'B', true);
ADIEncoder Robot::LeftEncoder('E','F', false);
ADIEncoder Robot::BackEncoder('C', 'D', false);


std::atomic<double> Robot::turn_offset_x = 0;
std::atomic<double> Robot::turn_offset_y = 0;
std::atomic<double> Robot::x = 0;
std::atomic<double> Robot::y = 0;



PD Robot::power_PD(.8, 5, 0);
PD Robot::strafe_PD(.8, .6, 0);
PD Robot::turn_PD(3, 1, 0);



std::map<std::string, std::unique_ptr<pros::Task>> Robot::tasks;

double offset_back = 6.50;
double offset_middle = 5.3;
double wheel_circumference = 2.75 * M_PI;


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
  }
}

void Robot::odometry(void *ptr) {
  double last_x = 0;
  double last_y = 0;
  double last_phi = 0;


  while(true) {
    double cur_phi = TO_RAD(Robot::InertialSensor.get_rotation());
    double dphi = cur_phi - last_phi;

    double cur_turn_offset_x = 360 * (offset_back * dphi) / wheel_circumference;
    double cur_turn_offset_y = 360 * (offset_middle * dphi) / wheel_circumference;
    /* Calculate how much the encoders have turned as a result of turning ONLY in order to
    isolate readings representing lateral or axial movement from readings representing
    turning in place */

    Robot::turn_offset_x = (float)Robot::turn_offset_x + cur_turn_offset_x;
    Robot::turn_offset_y = (float)Robot::turn_offset_y + cur_turn_offset_y;

    double cur_y = (((float)Robot::LeftEncoder.get_value() - (float)Robot::turn_offset_y) + ((float)Robot::RightEncoder.get_value() + (float)Robot::turn_offset_y)) / 2;
    double cur_x = (float)Robot::BackEncoder.get_value() - (float )Robot::turn_offset_x;

    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "TTOF: %f - TTOY: %f",  (float)Robot::turn_offset_y, (float)Robot::turn_offset_x);
    double dy = cur_y - last_y;
    double dx = cur_x - last_x;

    double global_dy = dy * std::cos(cur_phi) + dx * std::sin(cur_phi);
    double global_dx = dx * std::cos(cur_phi) - dy * std::sin(cur_phi);
/* Apply rotation matrix to dx and dy to calculate global_dy and global_dx. Is required because if the Robot moves
on an orientation that is not a multiple of 90 (i.e. 22 degrees), x and y encoder values do not correspond
exclusively to either x or y movement, but rather a little bit of both */

    Robot::y = (float)Robot::y + global_dy;
    Robot::x = (float)Robot::x + global_dx;

    pros::screen::print(pros::E_TEXT_MEDIUM, 3, "LE: %3d, RE: %3d, BE: %3d, TEST: %3d", Robot::LeftEncoder.get_value(), Robot::RightEncoder.get_value(), Robot::BackEncoder.get_value(),  Robot::BackEncoder.get_value());
    pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Y: %f - X: %f - IMU value: %f - TEST: %f", (float)Robot::y, (float)Robot::x, InertialSensor.get_rotation() + (7.0/360 * InertialSensor.get_rotation()), 5.1);

    // Robot::turn_offset_x = (float)Robot::turn_offset_x + cur_turn_offset_x;
    // Robot::turn_offset_y = (float)Robot::turn_offset_y + cur_turn_offset_y;
    last_y = cur_y;
		last_x = cur_x;
		last_phi = cur_phi;

    delay(5);

   }
}


void Robot::move_to(std::vector<double> pose, double stop_threshold, bool pure_pursuit, int flipout_timer, std::vector<double> speeds)
{
    double new_y = pose[0];
    double new_x = pose[1];
    double heading = pose[2];


    std::deque<double> motion;

    double y_error = new_y - y;
    double x_error = -(new_x - x);
    int coefficient = 0;
    double last_x = x;
    double last_y = y;
    std::string powered = "intakes";

    int time = 0;

    double heading2 = (heading < 0) ? heading + 360 : heading - 360;
    if (pure_pursuit) heading = (abs(InertialSensor.get_rotation() - heading) < abs(InertialSensor.get_rotation() - heading2)) ? heading : heading2;
    double imu_error = -(InertialSensor.get_rotation() - heading);
    /* Calculate inverse headings (i.e. 1 deg = -359 deg), then find which heading is closer to current heading. For
    example, moving to -358 deg would require almost a full 360 degree turn from 1 degree, but from its equivalent of -359
    deg, it only takes a minor shift in position */

    while (abs(y_error) > 10 || abs(x_error) > 10 || abs(imu_error) > 1)
    { /* while Robot::y, Robot::x and IMU heading are all more than the specified margin away from the target */

        if ((int)motion.size() == 10) motion.pop_front();
        motion.push_back(abs(last_x - Robot::x) + abs(last_y - Robot::y));
        double sum = 0;
        for (int i = 0; i < motion.size(); i++) sum += motion[i];
        double motion_average = sum / 10;
        if (motion_average < stop_threshold && time > 100) break;

        last_x = Robot::x;
        last_y = Robot::y;

        double phi = TO_RAD(InertialSensor.get_rotation());
        double power = power_PD.get_value(y_error * std::cos(phi) + x_error * std::sin(phi)) * speeds[0];
        double strafe = strafe_PD.get_value(x_error * std::cos(phi) - y_error * std::sin(phi)) * speeds[1];
        double turn = turn_PD.get_value(imu_error) * speeds[2];
        driveOdom(power, turn, strafe);
        /* Using our PD objects we use the error on each of our degrees of freedom (axial, lateral, and turning movement)
        to obtain speeds to input into Robot::mecanum. We perform a rotation matrix calculation to translate our y and x
        error to the same coordinate plane as Robot::y and Robot::x to ensure that the errors we are using are indeed
        proportional/compatible with Robot::y and Robot::x */

        imu_error = -(InertialSensor.get_rotation() - heading);
        y_error = new_y - Robot::y;
        x_error = -(new_x -Robot::x);
        /* Recalculating our error by subtracting components of our current position vector from target position vector */

        if (pure_pursuit) return;
        delay(5);
        time += 5;


    }
    reset_PD();
    Robot::brake("stop");
    pros::screen::print(pros::E_TEXT_MEDIUM, 7, "DONE");
}

void Robot::reset_PD() {
	power_PD.reset();
	strafe_PD.reset();
	turn_PD.reset();
}

void Robot::brake(std::string mode)
{

	if (mode.compare("coast") == 0)
	{
		LeftFrontWheel.set_brake_mode(E_MOTOR_BRAKE_COAST);
		RightBackWheel.set_brake_mode(E_MOTOR_BRAKE_COAST);
		LeftBackWheel.set_brake_mode(E_MOTOR_BRAKE_COAST);
		RightBackWheel.set_brake_mode(E_MOTOR_BRAKE_COAST);
	}
	else if (mode.compare("hold") == 0)
	{
		LeftFrontWheel.set_brake_mode(E_MOTOR_BRAKE_HOLD);
		RightFrontWheel.set_brake_mode(E_MOTOR_BRAKE_HOLD);
		LeftBackWheel.set_brake_mode(E_MOTOR_BRAKE_HOLD);
		RightBackWheel.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	}
	else LeftFrontWheel = RightFrontWheel = LeftBackWheel = RightBackWheel = 0;
}

void Robot::driveOdom(int power, int turn, int strafe)
{
  strafe = -strafe;


  int max_left_front_speed = ((power + turn + strafe) > 127) ? 127 : (power + turn + strafe);
  int max_left_back_speed = ((power + turn - strafe) > 127) ? 127 : (power + turn - strafe);
  int max_right_front_speed = ((power - turn - strafe) > 127) ? 127 : (power - turn - strafe);
  int max_right_back_speed = ((power - turn + strafe) > 127) ? 127 : (power - turn + strafe);



  Robot::LeftFrontWheel = max_left_front_speed;
  Robot::LeftBackWheel = max_left_back_speed;
  Robot::RightFrontWheel = max_right_front_speed;
  Robot::RightBackWheel = max_right_back_speed;
}


void Robot::drive(int power, int turn, int strafe)
{

  power = (abs(power) > 55) ? power : 0;
  turn = (abs(turn) > 55) ? turn : 0;
  strafe = (abs(strafe) > 55) ? -strafe : 0;

  int max_left_front_speed = ((power + turn + strafe) > 127) ? 127 : (power + turn + strafe);
  int max_left_back_speed = ((power + turn - strafe) > 127) ? 127 : (power + turn - strafe);
  int max_right_front_speed = ((power - turn - strafe) > 127) ? 127 : (power - turn - strafe);
  int max_right_back_speed = ((power - turn + strafe) > 127) ? 127 : (power - turn + strafe);



  Robot::LeftFrontWheel = max_left_front_speed;
  Robot::LeftBackWheel = max_left_back_speed;
  Robot::RightFrontWheel = max_right_front_speed;
  Robot::RightBackWheel = max_right_back_speed;
}

void Robot::spin(void *ptr) {
  while(true) {
    Robot::ShooterOne = -127;
    Robot::ShooterTwo = -127;
  }
}

void Robot::shoot() {
  Robot::pneumatics.set_value(1);
  delay(100);
  Robot::pneumatics.set_value(0);

}


void Robot::driveControl(void *ptr)
{


  while(true)
  {

    int power = Robot::Controller1.get_analog(ANALOG_LEFT_Y);
    int strafe = Robot::Controller1.get_analog(ANALOG_LEFT_X);
    int turn = Robot::Controller1.get_analog(ANALOG_RIGHT_Y);
    drive(power, turn, strafe);
    //FourBar

    if(Robot::Controller1.get_digital(DIGITAL_L1)) {
      Robot::RollerSpinner= 106;
      Robot::Intake = 114;
    }
    else if (Robot::Controller1.get_digital(DIGITAL_L2)) {
      Robot::RollerSpinner = -105;
      Robot::Intake = -114;
    }
    else {
      Robot::RollerSpinner = 0;
      Robot::Intake = 0;
      Robot::Intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    }

    if(Robot::Controller1.get_digital(DIGITAL_A)) {

      Robot::pneumatics.set_value(1);
    }
    else {
      Robot::pneumatics.set_value(0);
    }





    //Goal Spinner
    if(Robot::Controller1.get_digital(DIGITAL_R1)) {
      Robot::ShooterOne = -95;
      Robot::ShooterTwo = -95;
    }
    else if(Robot::Controller1.get_digital(DIGITAL_R2)) {
      Robot::ShooterOne = -120;
      Robot::ShooterTwo = -120;
    }
    else {
      Robot::ShooterOne.set_brake_mode(E_MOTOR_BRAKE_COAST);
      Robot::ShooterTwo.set_brake_mode(E_MOTOR_BRAKE_COAST);
      Robot::ShooterOne = 0;
      Robot::ShooterTwo = 0;
    }

  }
}
