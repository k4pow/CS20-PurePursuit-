#include "api.h"
#include "chassis.hpp"
#include "config.hpp"
#include "robot.hpp"
#include "autonomous.hpp"
#include "math.h"

//GLOBAL VARIABLES
//heading gives a value from -360-360 degrees
//converts to be 0-360 degrees i think
//converts into rad

//the inertial sensor is consistently inaccruate. a 360 turn is about 350 degrees
//it is consistent in its inaccuracy so
//i added a multiplier to get it to the actual correct heading
double innerL(){
  double angle = fmod(imuLeft.get_rotation()*1.027,360);
  while(angle <0)
  {
    angle+=360;
  }
  while(angle >360)
  {
    angle-=360;
  }
  return angle;
}
double innerR(){
  double angle =  fmod(imuRight.get_rotation()*1.013,360);
  while(angle <0)
  {
    angle+=360;
  }
  while(angle >360)
  {
    angle-=360;
  }
  return angle;
}
//returns heading in radians
double imuTheta(){
  double x = (cos(innerL()*M_PI/180 + M_PI) + cos(innerR()*M_PI/180 + M_PI)) / 2;
  double y = (sin(innerL()*M_PI/180 + M_PI) + sin(innerR()*M_PI/180 + M_PI)) / 2;

  return std::abs(atan2f(y, x) + M_PI);
}

void drive(){
  //Y-axis of left joystick and X-axis of right joystick
  int axis3 = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
  int axis1 = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

  //joysticks return values from [-127,127]
  //the voltage controller of the motor is from [-12000,12000]
  //12000/127 is the multiplier so max joysticks is 12000 mV
  //this is just for use control that i did for fun and to test motor directions
  int leftSpeed = (axis3+axis1)*12000/127;
  int rightSpeed = (axis3-axis1)*12000/127;
  printf("%i, %i\n", leftSpeed, rightSpeed);

  frontLeft.move_voltage(leftSpeed);
  backLeft.move_voltage(leftSpeed);
  frontRight.move_voltage(rightSpeed);
  backRight.move_voltage(rightSpeed);
}

//gloabal variables for odom
double encoderTheta = 0, totalEncoderTheta = 0;
double theta = 0, lastTheta = 0, lastLeft = 0, lastRight = 0, lastCenter = 0;
double pos_x = 0, pos_y = 0;
double theory = 0;
double deltaLeft = 0, deltaRight = 0, deltaCenter = 0, deltaTheta = 0, deltaTheory = 0;
double deltaX = 0, deltaY = 0;
double wheelTheta = 0;
int counter = 0;

//position tracking function
void odom(){

  theta = imuTheta();


  //get_position gets the rotation in centidegrees
  //arc length = theta(radians) * radius
  //there is a gear ratio in the rotation sensor 3:5 gear ratio, so one rotation in the sensor isnt 1 rotation in the wheel.
  double currentLeft = (rotLeft.get_position()/-100*3/5)*(1.375)*M_PI/180;
  double currentRight = (rotRight.get_position()/-100*3/5)*(1.375)*M_PI/180;
  double currentCenter = (rotCenter.get_position()/-100*3/5)*(1.375)*M_PI/180;

  //gets the theta of the robot with encoders instead the inertial sensor
  encoderTheta = ((deltaLeft - deltaRight)/10.6);//14.375 is the distance between both encoders (inches)

  //printf("%f %f %f %f \n", currentLeft, currentRight, currentCenter, theta*180/M_PI);
  //calculate the change in arclength for each tracker wheel and theta
  deltaLeft = currentLeft - lastLeft;
  deltaRight = currentRight - lastRight;
  deltaCenter = currentCenter - lastCenter;
  deltaTheta = theta - lastTheta;

  //theoretical wheel to keep position when the robot is doing a spot turn
  //when doing a spot turn, right and left wheels cancel each other out but the center wheel does not.
  deltaTheory = (deltaLeft-deltaRight)*0.3;
  //delta center + deltaTheory = 0
  theory+=deltaTheory;
  totalEncoderTheta +=encoderTheta;
  //printf("%f %f \n", currentCenter, theory);
  //calculate the change in x and y
  //this is essentially vectors and using trig you can calculate the change
  deltaX = (((deltaLeft + deltaRight)/2*1.0f * -sinf(-theta)) - ((deltaCenter + deltaTheory)*1.0f * -cosf(-theta)));
  deltaY = (((deltaLeft + deltaRight)/2*1.0f *  cosf(-theta)) - ((deltaCenter + deltaTheory)*.98f * -sinf(-theta)));

  //update the position of the robot
  pos_x +=deltaX;
  pos_y += deltaY;

  //get the previous variables to calculate future delta
  lastLeft  = currentLeft;
  lastRight = currentRight;
  lastCenter  = currentCenter;
  lastTheta = theta;
}

// Normalizes any number to a sets range
// by assuming the range wraps around when going below min or above max
//normalize keeps it within the range, but for Pure Pursuit, it will turn the theta
//into normal graph where 0 is on the positive x-axis, 90 is the positive y-axis etc.
//by adding 450 to imuTheta
double normalize(double value, double start, double end){
  double width = end - start;   //
  double offsetValue = value - start;   // value relative to 0

  return (offsetValue-(floor(offsetValue/width)*width))+start ;
  // + start to reset back to start of original range
}

//PID CONSTRUCTORS
PID::PID(double kP, double kD, double target){
  pkP = kP;
  pkD = kD;
  pTarget = target;
}

PID::PID(double kP, double kD, double targetX, double targetY, double maxSpeed, double preferredAngle){
  pkP = kP;
  pkD = kD;
  pTargetX = targetX;
  pTargetY = targetY;
  pMaxSpeed = maxSpeed;
  pPreferredAngle = preferredAngle;
}

//PID variables
bool toggle = true;
bool toggle2 = true;
double displacementX;
double displacementY;
double headingGoal;
double headingStart;
double headingCont;
double headingCart;
double turnPreviousError;
double turnCount;
double errorCount;
double turnError;
double forwardPreviousError;
double distanceCount;
double distanceError;
double turnIntegral;
double turnKP = 500; //turn correction proportional
double turnKI = 0; //turn correction integral
double turnKD = 50; //turn correction derivative
double distancePreviousError;
double xTarget;
double yTarget;

void PID::turnPID(){
  odom();
  toggle = true;
  while(1){
  odom();

  //print variables to the brain
  pros::lcd::print(5,"X: ""%f", pos_x);
	pros::lcd::print(6,"Y: ""%f", pos_y);
	pros::lcd::print(7,"THETA: " "%f", theta*180/M_PI);

  double headingStart;
  if(toggle == true){
    headingStart = imuTheta()*180/M_PI;
    // std::cout << "turnStart " << headingStart << std::endl;
    // std::cout << "turnTarget " << p_target << std::endl;
    toggle = false;
  }
  //headingGoal - imuTheta()*180/M_PI how much the robot needs to turn to face its headingGoal
  //these if statements optimize it as the robot only needs to turn 180 degrees maximum to reach any heading
  //but if the goal is to face 5 degrees and the robot is facing 355, the error is 355 degrees.
  if(pTarget - imuTheta()*180/M_PI > 180){
    turnError = pTarget-imuTheta()*180/M_PI-360;
  }
  if(-180 < pTarget - imuTheta() && pTarget - imuTheta()*180/M_PI <= 180){
    turnError = pTarget-imuTheta()*180/M_PI;
  }
  if(pTarget - imuTheta()*180/M_PI <= -180){
    turnError = pTarget - imuTheta()*180/M_PI + 360;
  }
  double turnDerivative = turnError - turnPreviousError;
  //std::cout << "tError " << turnError << std::endl;

  //if error is within +/- 2.5 degrees
  if (turnError < 2.5 && turnError > -2.5){
    turnCount += 1;
  } else { //If It isn't within +/- 2.5 degrees
    turnCount = 0;
  }
  //if its been within the tolerance for 100ms
  //code loops every 10ms
  if(turnCount >= 10){
    toggle = true;
    turnCount=0;
    stop();
    break;
  }

  //turning doesnt need a full PID, there just is no point in doing it, it works fine without
  //speed = error*multiplier
  //multiplier is the tuning value.
  double mVrequest = turnError * pkP + turnDerivative * pkD;
    frontLeft.move_voltage(mVrequest);
    backLeft.move_voltage(mVrequest);
    frontRight.move_voltage(-mVrequest);
    backRight.move_voltage(-mVrequest);
  turnPreviousError = turnError;
  }
}

void PID::StraightPID(){
  odom();
  toggle = true;
  toggle2 = true;
  while(1){
  odom();

  pros::lcd::print(5,"X: ""%f", pos_x);
	pros::lcd::print(6,"Y: ""%f", pos_y);
	pros::lcd::print(7,"THETA: " "%f", theta*180/M_PI);

  double xStart;
  double yStart;
  double calcDistance;
  xTarget = pTargetX;
  yTarget = pTargetY;
  if(toggle == true){
    xStart = pos_x;
    yStart = pos_y;
    calcDistance = sqrt(pow((xStart - xTarget),2) + pow((yStart - yTarget),2));
    headingStart = imuTheta()*180/M_PI;
    std::cout << "xStart " << xStart << std::endl;
    std::cout << "xTarget " << xTarget << std::endl;
    std::cout << "yStart " << yStart << std::endl;
    std::cout << "yTarget " << yTarget << std::endl;
    toggle = false;
  }
//For personal use and entering values, it makes more sense to use the clock method, but translations for pure pursuit is easier
//with cartesian plane
//headingCart is heading cartesion
  headingCart = normalize(-imuTheta()*180/M_PI+450, 0, 360);

  displacementX = xTarget - pos_x;
  displacementY = yTarget - pos_y;

  double displacementX_local = displacementY*cos(headingCart/180*3.1415926535) - displacementX*sin(headingCart/180*3.1415926535);
  double displacementY_local = displacementY*sin(headingCart/180*3.1415926535) + displacementX*cos(headingCart/180*3.1415926535);

  double movementXratio = displacementX_local/(fabs(displacementX_local)+fabs(displacementY_local));
  double movementYratio = displacementY_local/(fabs(displacementX_local)+fabs(displacementY_local));

  //basic pythagorean thing calculating hypotenuse
  distanceError = sqrt(pow((pos_x-xTarget),2) + pow((pos_y-yTarget),2));
  std::cout << "xRatio " << movementXratio << std::endl;
  std::cout << "yRatio " << movementYratio << std::endl;

  //if there isnt a preferredAngle
  if(pPreferredAngle == -1){
    //if robot is within 3 inches of the target point
    if(distanceError < 3  &&  toggle2 == true){
      headingCont = imuTheta()*180/M_PI; //headingCont is heading of robot in degrees
      toggle2 = false;
    }
    if(distanceError < 3){
      headingGoal = headingCont;
    }else{
    //get heading from opposite/hypotenuse then convert to degrees
    headingGoal = (normalize((atan2(yTarget - pos_y,xTarget - pos_x)*180/3.1415926) - 90, 0, 360) - 360)* -1;
    }
  }else{//the goal of the robot is the preferred angle
    headingGoal = pPreferredAngle;
  }

  //headingGoal - imuTheta()*180/M_PI how much the robot needs to turn to face its headingGoal
  //these if statements optimize it as the robot only needs to turn 180 degrees maximum to reach any heading
  //but if the goal is to face 5 degrees and the robot is facing 355, the error is 355 degrees.
  if(headingGoal - imuTheta()*180/M_PI > 180){
    turnError = headingGoal - imuTheta()*180/M_PI-360;
  }
  //simplest calculation is possible the error is just headingGoal-robotHeading
  if(-180 < headingGoal - imuTheta()*180/M_PI  &&  headingGoal - imuTheta()*180/M_PI <= 180){

    turnError = headingGoal-imuTheta()*180/M_PI;
  }
  //
  if(headingGoal - imuTheta()*180/M_PI <= -180){
    turnError = headingGoal-imuTheta()*180/M_PI+360;
  }

  double turnDerivative = turnError - turnPreviousError;
  double distanceDerivative = distanceError - distancePreviousError;

  if(distanceError < 10 && distanceError > -10){
    distanceCount += 1;
  }
  else { //If It isn't within +/- 10
    distanceCount = 0;
  }

  if (distanceError < 10 && distanceError > -10){
    errorCount += 1;
  }

  else { //If It isn't within +/- 10
    errorCount = 0;
  }
  //end PID loop, sometimes the PID will get stuck in a loop forever.
  if(distanceCount>=30 || errorCount>=80){
    distanceCount=0;
    errorCount=0;
    toggle = true;
    stop();
    break;
  }

  if(turnError < 1 && turnError > -1){
    turnIntegral = 0; //I don't need integral if the turn error is within +/- 1
  }else{//If It isn't within +/- 1
    turnIntegral += turnError;
  }
  //calculating the speeds for the motors. turn error has been calculated and drive error has been calculated
  //in the most basic PID speed = error. There is a multiplier added for tuning (kP, kI, kD) hence PID.
  double mVdistance = distanceError * pkP + distanceDerivative * pkD;
  //std::cout << "mVdistance " << mVdistance << std::endl;
  double mVforward = movementYratio*mVdistance;
  double mVturn = turnError * turnKP + turnDerivative * turnKD + turnIntegral * turnKI;
  double maxCtrl =
  fmax(fmax(fabs(-mVforward + mVturn), fabs(mVforward + mVturn)),
  fmax(fabs(mVforward - mVturn), fabs(-mVforward - mVturn)));
  //maxCtrl is finding the max speed of the speed values calculated.
  //each motor has its respective speed, and if its speed goes above 12000 then the motor has maxed out
  //if its greater, then it caps the speed

  if(maxCtrl>12000){
    frontLeft.move_voltage((-mVforward + mVturn) / (maxCtrl / 12000)*pMaxSpeed);
    backLeft.move_voltage((mVforward + mVturn) / (maxCtrl / 12000)*pMaxSpeed);
    frontRight.move_voltage((mVforward - mVturn) / (maxCtrl / 12000)*pMaxSpeed);
    backRight.move_voltage((-mVforward - mVturn) / (maxCtrl / 12000)*pMaxSpeed);

  } else {//just default to what the code wants
    frontLeft.move_voltage((-mVforward + mVturn)*pMaxSpeed);
    backLeft.move_voltage((mVforward + mVturn)*pMaxSpeed);
    frontRight.move_voltage((mVforward - mVturn)*pMaxSpeed);
    backRight.move_voltage((-mVforward - mVturn)*pMaxSpeed);
  }

  turnPreviousError = turnError;
  distancePreviousError = distanceError;
  }
}

//------------------------------------------//

//purePursuit variables
double ppDisplacementX;
double ppDisplacementY;
double ppDistanceError;
double ppTurnError;
double ppHeadingGoal;
double ppHeadingCont;
double ppTurnPreviousError;
double ppTurnKP = 1000;
double ppTurnKD = 0;
double ppHeadingCart;
bool ppFinalPoint = false;

//class constructor
APPC::APPC(std::vector<std::array<double,3>> pathVector, int resolution, double lookaheadDistance){
  pPathVector = pathVector; //when calling the class this is the vector of points (x,y,theta) because of my limited drive, theta is-1
  pResolution = resolution; //Resolution of the points generated. distance between each point
  pLookaheadDistance = lookaheadDistance; //how far the robot looks ahead to the path
}

void APPC::PurePursuit(){

  std::array<double,3> targetPoint = {0,0,0};
  std::vector<std::array<double,3>> subpoints;
  //std::cout << "Subpoint Calculations" << std::endl;
  //Subpoint Calculations
  for(int i = 0; i < pPathVector.size() - 1; i++){
    //gets the starting coordinates and ending coordinates from user input in main.cpp
    std::array<double,3> coordStart = {std::get<0>(pPathVector[i]),std::get<1>(pPathVector[i]),std::get<2>(pPathVector[i])};
    std::array<double,3> coordEnd = {std::get<0>(pPathVector[i+1]),std::get<1>(pPathVector[i+1]),std::get<2>(pPathVector[i+1])};
    double xDistance = coordEnd[0]-coordStart[0];
    double yDistance = coordEnd[1]-coordStart[1];

    subpoints.push_back(coordStart);

    //std::cout << coordStart[0] << ", " << coordStart[1] << std::endl;
    //point generation
    for(int i2=1; i2<pResolution; i2++){

      std::array<double,3> newPoint =
      {coordStart[0]+(xDistance/pResolution*i2),coordStart[1]+(yDistance/pResolution*i2),coordStart[2]};

      subpoints.push_back(newPoint);

      //std::cout << newPoint[0] << ", " << newPoint[1] << std::endl;
    }
    //last coordStart
    if(i == pPathVector.size()-2){
      subpoints.push_back(coordEnd);
      //std::cout << coordEnd[0] << ", " << coordEnd[1] << std::endl;
    }
  }
  int i3;

  // std::cout << "Amount of Points: " << subpoints.size() << std::endl;
  // std::cout << std::endl;
  // std::cout << "Target Points: " << std::endl;

  std::array<double,3> previousPoint = {std::get<0>(pPathVector[0]),std::get<0>(pPathVector[1]),std::get<0>(pPathVector[2])};
  double targetDistance = pLookaheadDistance+10;

  while(1){
    odom();

    pros::lcd::print(5,"X: ""%f", pos_x);
  	pros::lcd::print(6,"Y: ""%f", pos_y);
  	pros::lcd::print(7,"THETA: " "%f", theta*180/M_PI);

    double previousSubpointDistance=1000;
    int targetPosition;
    int previousTargetPosition = 10;

    i3+=1;

    double finalDistance = sqrt(pow((pos_x-std::get<0>(pPathVector[pPathVector.size()-1])),2)+pow((pos_y-std::get<1>(pPathVector[pPathVector.size()-1])),2));

    //Find the closest point
    //loop through all the subpoints and find the closest one.
    if(finalDistance > pLookaheadDistance+1){
      for(int i=0; i<subpoints.size(); i++){
        double subpointX = std::get<0>(subpoints[i]);
        double subpointY = std::get<1>(subpoints[i]);
        double subpointHeading = std::get<2>(subpoints[i]);
        double subpointDistance = sqrt(pow((pos_x-subpointX),2)+pow((pos_y-subpointY),2));
        double subpointDistanceDifference = subpointDistance-pLookaheadDistance;
        double previousSubpointDifference = previousSubpointDistance-pLookaheadDistance;
        if(subpointDistanceDifference < previousSubpointDifference && subpointDistanceDifference > 0){
          previousSubpointDistance = subpointDistance;
          targetPoint = {subpointX,subpointY,subpointHeading};
          targetPosition = i;
        }
      }

      for(int i = 0; i < targetPosition; i++){
        if(i!=subpoints.size()-1){
          std::get<0>(subpoints[i]) = -1000;
          std::get<1>(subpoints[i]) = -1000;
          std::get<2>(subpoints[i]) = 0;
        }
      }

      if(targetPosition >= previousTargetPosition+4 && targetDistance > pLookaheadDistance){
        targetPoint = previousPoint;
      }
      targetDistance = sqrt(pow((pos_x-targetPoint[0]),2)+pow((pos_y-targetPoint[1]),2));

      previousPoint = targetPoint;
      previousTargetPosition = targetPosition;
      }else {
      ppFinalPoint = true;
      targetPoint = {std::get<0>(subpoints[subpoints.size()-1]),std::get<1>(subpoints[subpoints.size()-1]),std::get<2>(subpoints[subpoints.size()-1])};
    }

    //Movement Section head to the nearest point
    double pp_turnDerivative = ppTurnError - ppTurnPreviousError;
    //Vectors
    //heading of the robot in cartesian
    ppHeadingCart = normalize(-imuTheta()+450, 0, 360);

    //deltaX and deltaY
    ppDisplacementX = targetPoint[0]-pos_x;
    ppDisplacementY = targetPoint[1]-pos_y;

    //find the displacement relative to the robot because it may not be facing the points
    //there are 2 vectors, how much to go forwards, how much to strafe
    double ppDisplacementXLocal =
    ppDisplacementY*cos(ppHeadingCart/180*3.1415926535) - ppDisplacementX*sin(ppHeadingCart/180*3.1415926535);
    //-50
    double ppDisplacementYLocal =
    ppDisplacementY*sin(ppHeadingCart/180*3.1415926535) + ppDisplacementX*cos(ppHeadingCart/180*3.1415926535);

    //quantifying the vecotr, numbers between 0 and 1
    double pp_movementXratio = ppDisplacementXLocal/(fabs(ppDisplacementXLocal)+fabs(ppDisplacementYLocal));
    double pp_movementYratio = ppDisplacementYLocal/(fabs(ppDisplacementXLocal)+fabs(ppDisplacementYLocal));

    //simple pythag
    ppDistanceError = sqrt(pow((pos_x-targetPoint[0]),2)+pow((pos_y-targetPoint[1]),2));

    //Output every 50ms i was using it for testing.
    if(i3>=5){
      // std::cout << "Target: " <<targetPoint[0] << ", " << targetPoint[1] << ", " << targetPoint[2] << std::endl;
      // std::cout << "Distance: " << targetDistance << std::endl;
      i3=0;
    }

    //Heading is undefined theres no desired robot heading so it can calculate what it wants.
    if(targetPoint[2] == -1){
      ppHeadingGoal = (normalize((atan2(targetPoint[1]-pos_y,targetPoint[0]-pos_x)*180/3.1415926)-90, 0, 360)-360)*-1;
    } else {
      ppHeadingGoal = targetPoint[2];
    }

    //optimize the direction to turn like before
    if(ppHeadingGoal-theta > 180){
      ppTurnError = ppHeadingGoal-theta-360;
    }
    if(-180 < ppHeadingGoal - theta && ppHeadingGoal - theta <= 180){
      ppTurnError = ppHeadingGoal-theta;
    }
    if(ppHeadingGoal - theta <= -180){
      ppTurnError = ppHeadingGoal-theta+360;
    }

    if(ppFinalPoint == true){//it has almost reached its final point, turn on the PID to stop
      PID PathFollowing(5000, 500, targetPoint[0], targetPoint[1], 1, targetPoint[2]);
      PathFollowing.StraightPID();

      ppFinalPoint = false;
      subpoints.clear();
      pPathVector.clear();
      break;
    }

    //Motor Control
    double turnPower = ppTurnError * ppTurnKP + pp_turnDerivative * ppTurnKD;
    //please ignore the really bad formatting, i needed it to fit on my screen
    double pp_maxCtrl =
    fmax(fmax(
    fabs(-pp_movementYratio + turnPower/12000 + pp_movementXratio),
    fabs(pp_movementYratio + turnPower/12000 + pp_movementXratio)
    ),
    fmax(
    fabs(pp_movementYratio - turnPower/12000 + pp_movementXratio),
    fabs(-pp_movementYratio - turnPower/12000 + pp_movementXratio)));

    //terminal output for debugging
    printf("FL: %f\n BL: %f\n FR: %f\n BR: %f\n\n",
    -((-pp_movementYratio + turnPower/12000 + pp_movementXratio)*12000/pp_maxCtrl),
    (pp_movementYratio + turnPower/12000 + pp_movementXratio)*12000/pp_maxCtrl,
    (pp_movementYratio - turnPower/12000 + pp_movementXratio)*12000/pp_maxCtrl,
    -((-pp_movementYratio - turnPower/12000 + pp_movementXratio)*12000/pp_maxCtrl));

    if(pp_maxCtrl > 1){
      frontLeft.move_voltage(-((-pp_movementYratio + turnPower/12000 + pp_movementXratio)*12000/pp_maxCtrl));
      backLeft.move_voltage((pp_movementYratio + turnPower/12000 + pp_movementXratio)*12000/pp_maxCtrl);

      frontRight.move_voltage((pp_movementYratio - turnPower/12000 + pp_movementXratio)*12000/pp_maxCtrl);
      backRight.move_voltage(-((-pp_movementYratio - turnPower/12000 + pp_movementXratio)*12000/pp_maxCtrl));

    }else {
      frontLeft.move_voltage(-((-pp_movementYratio + turnPower/12000 + pp_movementXratio)*12000));
      backLeft.move_voltage((pp_movementYratio + turnPower/12000 + pp_movementXratio)*12000);

      frontRight.move_voltage((pp_movementYratio - turnPower/12000 + pp_movementXratio)*12000);
      backRight.move_voltage(-((-pp_movementYratio - turnPower/12000 + pp_movementXratio)*12000));

    }

    //Variables to get derivatives for next loop. change in unit/10ms
    previousPoint = targetPoint;
    previousTargetPosition = targetPosition;
    ppTurnPreviousError = ppTurnError;
    pros::delay(10);
  }
}


//stop the drivetrain
void stop(){
  frontLeft.move_velocity(0);
  frontRight.move_velocity(0);
  backLeft.move_velocity(0);
  backRight.move_velocity(0);
}
