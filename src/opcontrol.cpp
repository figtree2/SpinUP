#include "main.h"
#include "Robot.h"


void opcontrol() {
	Robot::start_task("Display", Robot::display);
	Robot::start_task("Driver", Robot::driveControl);
}
