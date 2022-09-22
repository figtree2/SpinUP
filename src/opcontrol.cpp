#include "main.h"
#include "Robot.h"


void opcontrol() {
	Robot::start_task("Driver", Robot::driveControl);
	Robot::start_task("odom", Robot::odometry);
}
