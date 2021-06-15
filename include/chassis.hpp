#ifndef CHASSIS_HPP
#define CHASSIS_HPP
#include "api.h"

void drive();
void point(double targetX, double targetY, double targetTheta, double driveSpeed, double turnSpeed, double driveRate, double turnRate, bool twoinch ,  int intakeTimer);
void pointCoast(double targetX, double targetY, double targetTheta, double driveSpeed, double turnSpeed, double driveRate, double turnRate, bool twoinch, int intakeTimer, bool negative, bool indexBreak);
void coasting( double targetX, double targetY, double targetTheta, double driveSpeed, double turnSpeed, double driveRate, double turnRate, bool twoinch, int intakeTimer, bool negative);
void odom();
void stop();
void odomZero();
double normalize();
double odom2();
double imuTheta();

extern int coastSpeed;
extern double deltaTheory;
extern double theory;
extern double deltaCenter;
extern double currentLeft, lastLeft, deltaLeft;
extern double currentRight, lastRight, deltaRight;
extern double currentCenter, lastCenter, deltaCenter;
extern float inertL;
extern float inertR;
extern double theta, deltaTheta;
extern int counter;
extern double pos_x;
extern double pos_y;
extern bool inPositionL;
extern bool inPositionR;
extern double tolerance;
extern double deltaX, deltaY;
extern double derivative;
extern double totalEncoderTheta;
extern  double encoderTheta;
extern double lastEncoderTheta;
extern double deltaEncoderTheta;
extern double innerL();
extern double innerR();
extern double lastTheta;
extern double lastLeft;
extern double lastRight;
extern double lastCenter;
extern double pos_x;
extern double pos_y;
extern double wheelTheta;

//classes
class PID{
  double pkP;
  double pkD;
  double pTarget;
  double pTargetX;
  double pTargetY;
  double pMaxSpeed;
  double pPreferredAngle;

  public:
    PID(double kP, double kD, double target);

    PID(double kP, double kD, double targetX, double targetY, double maxSpeed, double preferredAngle);

    void turnPID();
    void StraightPID();

};

class APPC{

  std::vector<std::array<double,3>> pPathVector;
  int pResolution;
  double pLookaheadDistance;
  bool pMoreIntake;

  public:
  APPC(std::vector<std::array<double,3>> pathVector, int resolution, double lookaheadDistance);
  void PurePursuit();

};
#endif
