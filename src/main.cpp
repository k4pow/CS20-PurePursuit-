#include "main.h"
#include "autonomous.hpp"
#include "config.hpp"
#include <iostream>
#include "math.h"
#include "chassis.hpp"
#include "robot.hpp"
#include "api.h"
#include "iostream"
#include <fstream>
#include <string>
#include "pros/rotation.h"

/*
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize(){

	pros::lcd::initialize();
  imuLeft.reset();
  imuRight.reset();
  pros::delay(3000);

	rotLeft.reset_position();
	rotRight.reset_position();
	rotCenter.reset_position();

	master.rumble(". .");



	frontLeft.set_brake_mode(MOTOR_BRAKE_COAST);
	frontRight.set_brake_mode(MOTOR_BRAKE_COAST);
	backLeft.set_brake_mode(MOTOR_BRAKE_COAST);
	backRight.set_brake_mode(MOTOR_BRAKE_COAST);

}
/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {

}

void display(){

	pros::lcd::print(5,"X: ""%f", pos_x);
	pros::lcd::print(6,"Y: ""%f", pos_y);
	pros::lcd::print(7,"THETA: " "%f", theta*180/M_PI);

	printf("(%f %f) (%f, %f) (%f)  \n",
	innerL(),
	innerR(),
	pos_x, pos_y, theta);

//	printf("%f %f %f \n", imuLeft.get_heading(), imuRight.get_heading(), theta);
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {

}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {

	 std::vector<std::array<double,3>> pathVector1 = {{0,0,-1},{5,40,-1},{20,20,-1},{20,10,-1}};
	 APPC path1(pathVector1,7,7);
	 path1.PurePursuit();

}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	while(1){
		drive();
		display();
		odom();
		counter++;
		pros::delay(10);
	}
}
