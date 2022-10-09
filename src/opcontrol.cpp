#include "main.h"
#include "Robot.h"


void initialize() {
	pros::lcd::initialize();
	Robot::InertialSensor.reset();
}
void opcontrol() {
	Robot::start_task("Odom", Robot::odometry);
	Robot::start_task("Driver", Robot::driveControl);
}
