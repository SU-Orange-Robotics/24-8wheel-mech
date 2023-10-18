#include "vex.h"
#include "drive.h"
#include "robot-config.h"

using namespace vex;


Drive::Drive() {    
    originCorr = 0;
    originHeading = IMU.heading();
}

void Drive::drive(double y, double x, double theta) {

    inputAdjust(y, x, theta);

    fieldRelativize(y, x, theta);

    //some of these +'s and -'s will need to be changed when the wheels are flipped around
    frontLeftA.spin(directionType::fwd, y + x + theta, velocityUnits::pct); 
    frontLeftB.spin(directionType::fwd, y + x + theta, velocityUnits::pct);

    backRightA.spin(directionType::fwd, y + x - theta, velocityUnits::pct);
    backRightB.spin(directionType::fwd, y + x - theta, velocityUnits::pct);

    frontRightA.spin(directionType::fwd, y - x - theta, velocityUnits::pct);
    frontRightB.spin(directionType::fwd, y - x - theta, velocityUnits::pct);

    backLeftA.spin(directionType::fwd, y - x + theta, velocityUnits::pct);
    backLeftB.spin(directionType::fwd, y - x + theta, velocityUnits::pct);
    
}

void Drive::inputAdjust(double &fwd, double &str, double &theta) {
    fwd *= abs(fwd) < deadzone ? 0.0 : 1.0;
    str *= abs(str) < deadzone ? 0.0 : 1.0;
    theta *= abs(theta) < deadzone ? 0.0 : 1.0;
    
    fwd /= 100;
    str /= 100;
    theta /= 100;
    
    /*
    fwd = pow(fwd, 2) * ((fwd > 0) ? 1 : -1);
    str = pow(str, 2) * ((str > 0) ? 1 : -1);
    theta = pow(theta, 2) * ((theta > 0) ? 1 : -1);
    */

    fwd *= 100;
    str *= 50; // effectively halves max turning values
    theta *= 100;

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