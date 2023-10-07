/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       zhuowz                                                    */
/*    Created:      10/6/2023, 6:48:52 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

// 8 motor 4 WHEEL mechanum drive
motor frontLeftA(PORT2, gearSetting::ratio6_1, false);
motor frontLeftB(PORT3, gearSetting::ratio6_1, false); //doesn't work due to mechanical issue
motor frontRightA(PORT9, gearSetting::ratio6_1, true);
motor frontRightB(PORT10, gearSetting::ratio6_1, true);
motor backLeftA(PORT19, gearSetting::ratio6_1, false);
motor backLeftB(PORT20, gearSetting::ratio6_1, false);
motor backRightA(PORT6, gearSetting::ratio6_1, true);
motor backRightB(PORT5, gearSetting::ratio6_1, true);

controller Controller1(controllerType::primary);

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//

static void drive(double y, double x, double theta) {
    x /= 100;
    y /= 100;
    theta /= 100;
    
    x = pow(x, 2) * ((x > 0) ? 1 : -1);
    y = pow(y, 2) * ((y > 0) ? 1 : -1);
    theta = pow(theta, 2) * ((theta > 0) ? 1 : -1);

    x *= 100;
    y *= 100;
    theta *= 100;

    //if (x + y + theta >= 3) {

    //some of these +'s and -'s will need to be changed when the wheels are flipped around
    frontLeftA.spin(directionType::fwd, y + x + theta, velocityUnits::pct); 
    frontLeftB.spin(directionType::fwd, y + x + theta, velocityUnits::pct);

    backRightA.spin(directionType::fwd,  y + x - theta, velocityUnits::pct);
    backRightB.spin(directionType::fwd,  y + x - theta, velocityUnits::pct);

    frontRightA.spin(directionType::fwd, y - x - theta, velocityUnits::pct);
    frontRightB.spin(directionType::fwd, y - x - theta, velocityUnits::pct);

    backLeftA.spin(directionType::fwd,  y - x + theta, velocityUnits::pct);
    backLeftB.spin(directionType::fwd,  y - x + theta, velocityUnits::pct);
    //}
  }


int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    // 8 motor 4 WHEEL mechanum drive
    
    drive(Controller1.Axis3.value(), Controller1.Axis4.value(), Controller1.Axis1.value());

    wait(10, msec);
  }
}
