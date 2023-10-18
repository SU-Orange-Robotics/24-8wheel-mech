#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

// 8 motor 4 WHEEL mechanum drive
motor frontLeftA(PORT2, gearSetting::ratio6_1, false);
motor frontLeftB(PORT3, gearSetting::ratio6_1, false);
motor frontRightA(PORT9, gearSetting::ratio6_1, true);
motor frontRightB(PORT10, gearSetting::ratio6_1, true);
motor backLeftA(PORT19, gearSetting::ratio6_1, false);
motor backLeftB(PORT20, gearSetting::ratio6_1, false);
motor backRightA(PORT6, gearSetting::ratio6_1, true);
motor backRightB(PORT5, gearSetting::ratio6_1, true);

controller Controller1(controllerType::primary);
inertial IMU(PORT21);