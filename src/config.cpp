#include "config.hpp"
#include "api.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);

//drivetrain motors
pros::Motor frontLeft(15, pros::E_MOTOR_GEARSET_06, false);
pros::Motor frontRight(2, pros::E_MOTOR_GEARSET_06, true);
pros::Motor backLeft(13, pros::E_MOTOR_GEARSET_06, false);
pros::Motor backRight(1, pros::E_MOTOR_GEARSET_06, true);

//rotation sensor
pros::Rotation rotLeft(20);
pros::Rotation rotRight(3);
pros::Rotation rotCenter(7);

//intertial sensors
pros::Imu imuLeft(16);
pros::Imu imuRight(9);
