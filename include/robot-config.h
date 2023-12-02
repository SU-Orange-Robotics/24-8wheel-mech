#include "vex.h"
#include "drive.h"

using namespace vex;

// A global instance of competition
extern competition Competition;

// define your global instances of motors and other devices here

// 8 motor 4 WHEEL mechanum drive
extern motor frontLeft;
extern motor frontRight;
extern motor backLeft;
extern motor backRight;

extern motor intake;

extern motor catapultA;
extern motor catapultB;
extern motor catapultC;

extern rotation catapultRot;

extern controller Controller1;
extern inertial IMU;
extern gps gps1;

extern Drive drive;