#include "Robot.h"
#include "main.h"

using namespace std;
using namespace pros;
#define TO_RAD(n) n * M_PI / 180;


Controller Robot::Controller1(E_CONTROLLER_MASTER);
Motor Robot::LeftBackWheel(19);
Motor Robot::RightBackWheel(10, true);
Motor Robot::LeftFrontWheel(1);
Motor Robot::RightFrontWheel(14, true);

Motor Robot::ShooterOne(12);
Motor Robot::ShooterTwo(11, true);
Motor Robot::RollerSpinner(8);
Motor Robot::Intake(18);

pros::GPS Robot::GPSSensor(2);

IMU Robot::InertialSensor(17);

ADIDigitalOut Robot::pneumatics('A');
pros::c::adi_encoder_t Robot::LeftEncoder('B');
pros::c::adi_encoder_t Robot::RightEncoder('C');
pros::c::adi_encoder_t Robot::BackEncoder('D');
std::atomic<double> Robot::x = 0;
std::atomic<double> Robot::y = 0;



// PD Robot::power_PD(.45, .1);
// PD Robot::strafe_PD(.45, .1);
// PD Robot::turn_PD(3, 0);



std::map<std::string, std::unique_ptr<pros::Task>> Robot::tasks;

double offset_back = 1;
double offset_middle = 1;
double wheel_circumference = 1;

double Robot::turn_offset = 9.01;


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

void Robot::odometry(void *ptr) {
  double last_x = 0;
  double last_y = 0;
  double last_phi = 0;

  while(true) {
    double cur_phi = TO_RAD(Robot::InertialSensor.get_rotation());
    double dphi = cur_phi - last_phi;
    lcd::print(1, "phi: %3f", cur_phi);

    // double cur_turn_offset_x = 360 * (offset_back * dphi) / wheel_circumference;
    // double cur_turn_offset_y = 360 * (offset_middle * dphi) / wheel_circumference;
    double cur_turn_offset = 360 * (turn_offset * dphi) / wheel_circumference;

    // Robot::turn_offset_x = (float)Robot::turn_offset_x + cur_turn_offset_x;
    // Robot::turn_offset_y = (float)Robot::turn_offset_y + cur_turn_offset_y;

    turn_offset = (float)turn_offset + cur_turn_offset;

    lcd::print(3, "Offset: %3f", turn_offset);

    double left_front_encoder = LeftFrontWheel.get_position();
    double right_front_encoder = RightFrontWheel.get_position();
    double left_back_encoder = LeftBackWheel.get_position();
    double right_back_encoder = RightBackWheel.get_position();

    lcd::print(4, "LF: %3f, RF: %3f, LB: %3f, RB: %3f", left_front_encoder, right_front_encoder, left_back_encoder, right_back_encoder);

    // double cur_y = (((pros::c::adi_encoder_get(Robot::LeftEncoder) - turn_offset_y) + (pros::c::adi_encoder_get(RightEncoder) + turn_offset_x)) /2);
    // double cur_x = pros::c::adi_encoder_get(Robot::BackEncoder) - Robot::turn_offset_x;

    // double dy = cur_y - last_y;
    // double dx = cur_x - last_x;
    //
    // double global_dy = dy * std::cos(cur_phi) + dx * std::sin(cur_phi);
    // double global_dx = dx * std::cos(cur_phi) - dy* std::sin(cur_phi);
    //
    // Robot::y = (float)Robot::y + global_dy;
    // Robot::x = (float)Robot::x + global_dx;
    //
    // last_y = cur_y;
    // last_x = cur_x;
    // last_phi = cur_phi;
   }
}

void Robot::drive(int power, int turn, int strafe)
{

  power = (abs(power) > 55) ? power : 0;
  turn = (abs(turn) > 55) ? turn : 0;
  strafe = (abs(strafe) > 55) ? strafe : 0;

  int max_left_front_speed = ((power + turn + strafe) > 127) ? 127 : (power + turn + strafe);
  int max_left_back_speed = ((power + turn - strafe) > 127) ? 127 : (power + turn - strafe);
  int max_right_front_speed = ((power - turn - strafe) > 127) ? 127 : (power - turn - strafe);
  int max_right_back_speed = ((power - turn + strafe) > 127) ? 127 : (power - turn + strafe);



  Robot::LeftFrontWheel = max_left_front_speed;
  Robot::LeftBackWheel = max_left_back_speed;
  Robot::RightFrontWheel = max_right_front_speed;
  Robot::RightBackWheel = max_right_back_speed;
}


void Robot::driveControl(void *ptr)
{
  while(true)
  {

    int power = Robot::Controller1.get_analog(ANALOG_LEFT_Y);
    int strafe = Robot::Controller1.get_analog(ANALOG_LEFT_X);
    int turn = Robot::Controller1.get_analog(ANALOG_RIGHT_X);

    lcd::print(2, "power: %3f", power);
    drive(power, turn, strafe);
    //FourBar

    if(Robot::Controller1.get_digital(DIGITAL_L1)) {
      Robot::RollerSpinner= 106;
      Robot::Intake = 110;
    }
    else if (Robot::Controller1.get_digital(DIGITAL_L2)) {
      Robot::RollerSpinner = -105;
      Robot::Intake = -110;
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
      Robot::ShooterOne = 95;
      Robot::ShooterTwo = 95;
    }
    else if(Robot::Controller1.get_digital(DIGITAL_R2)) {
      Robot::ShooterOne = -95;
      Robot::ShooterTwo = -95;
    }
    else {
      Robot::ShooterOne = 0;
      Robot::ShooterTwo = 0;
    }

  }
}
