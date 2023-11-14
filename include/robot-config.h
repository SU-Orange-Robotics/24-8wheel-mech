#include "vex.h"
#include "drive.h"

using namespace vex;

// A global instance of competition
extern competition Competition;

// define your global instances of motors and other devices here

// 8 motor 4 WHEEL mechanum drive
extern motor frontLeftA;
extern motor frontLeftB;
extern motor frontRightA;
extern motor frontRightB;
extern motor backLeftA;
extern motor backLeftB;
extern motor backRightA;
extern motor backRightB;

extern controller Controller1;
extern inertial IMU;
extern gps gps1;

extern Drive drive;