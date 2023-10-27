#include "vex.h"
#include "drive.h"
#include "robot-config.h"

using namespace vex;

Drive::Drive() {    
    originCorr = 0;
    originHeading = IMU.heading();
    invertDrive = false;
    relativeDrive = false;
}

void Drive::drive(double fwd, double str, double theta) {

    //inputAdjust(fwd, str, theta);

    //fieldRelativize(fwd, str, theta);
    theta /= 2; //temporary

    //some of these +'s and -'s will need to be changed when the wheels are flipped around
    frontLeftA.spin(directionType::fwd, fwd + str + theta, velocityUnits::pct); 
    frontLeftB.spin(directionType::fwd, fwd + str + theta, velocityUnits::pct);

    backLeftA.spin(directionType::fwd, fwd - str + theta, velocityUnits::pct);
    backLeftB.spin(directionType::fwd, fwd - str + theta, velocityUnits::pct);

    frontRightA.spin(directionType::fwd, fwd - str - theta, velocityUnits::pct);
    frontRightB.spin(directionType::fwd, fwd - str - theta, velocityUnits::pct);

    backRightA.spin(directionType::fwd, fwd + str - theta, velocityUnits::pct);
    backRightB.spin(directionType::fwd, fwd + str - theta, velocityUnits::pct);

}

void Drive::inputAdjust(double &fwd, double &str, double &theta) {
    fwd *= fabs(fwd) < deadzone ? 0.0 : 1.0;
    str *= fabs(str) < deadzone ? 0.0 : 1.0;
    theta *= fabs(theta) < deadzone ? 0.0 : 1.0;
    
    fwd /= 100;
    str /= 100;
    theta /= 100;
    
    /*
    fwd = pow(fwd, 2) * ((fwd > 0) ? 1 : -1);
    str = pow(str, 2) * ((str > 0) ? 1 : -1);
    theta = pow(theta, 2) * ((theta > 0) ? 1 : -1);
    */

    fwd *= 100;
    str *= 100; 
    theta *= 50; // rcwScale reduces the max turning input

    //if statement to invert control direction
    if (invertDrive) {
        fwd *= -1;
        str *= -1;
    }
}

void Drive::fieldRelativize(double &fwd, double &str, double &theta) {
    originCorr = IMU.heading() + (360 - originHeading);

    // can toggle the relative drive functionality
    if (relativeDrive) {
        double originCorrection = toRadians(originHeading - IMU.heading());
        double temp = fwd * cos(originCorrection) - str * sin(originCorrection);
        str = str * cos(originCorrection) + fwd * sin(originCorrection);
        fwd = temp;
    }
    
}

void Drive::resetHeading() {
    originHeading = IMU.heading();
}