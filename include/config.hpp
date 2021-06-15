#ifndef CONFIG_H
#define CONFIG_H

#include "api.h"
extern pros::Controller master;

extern pros::Motor backLeft;
extern pros::Motor backRight;
extern pros::Motor frontLeft;
extern pros::Motor frontRight;

extern pros::Rotation rotLeft;
extern pros::Rotation rotRight;
extern pros::Rotation rotCenter;

extern pros::Imu imuLeft;
extern pros::Imu imuRight;

#endif
