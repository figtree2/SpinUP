#include "main.h"
#include "Robot.h"

using namespace pros;


void autonomous() {
  Robot::start_task("Odom", Robot::odometry);
  // Robot::start_task("Shoot", Robot::spin);
  // delay(1900);
  // Robot::shoot();
   Robot::move_to({600, 0, 90});
   delay(100);
   Robot::move_to({600, 600, 180});
   delay(100);
   Robot::move_to({000, 600, 270});
   delay(100);
   Robot::move_to({0, 0, 360});

  // Robot::move_to({600, 600, 40});
  // Robot::move_to({0,0,0});
  // Robot::move_to({600, 600, 0});
  // Robot::move_to({0,0,0});
  // Robot::move_to({600, 600, 0});
  // Robot::move_to({0,0,0});
  // Robot::move_to({600, 600, 0});
  // Robot::move_to({0,0,0});
  // Robot::move_to({400, 200, 0});
  // Robot::move_to({600, 600, 90});
  // Robot::move_to({0, 0, 0});


}
