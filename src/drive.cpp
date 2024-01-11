#include "vex.h"
#include "drive.h"
#include "robot-config.h"

using namespace vex;

Drive::Drive() {    
    originCorr = 0;
    originHeading = IMU.heading();
    invertDrive = false;
    relativeDrive = false;
    activePID = false;
}

void Drive::drive(double fwd, double str, double theta) {

    inputAdjust(fwd, str, theta);

    fieldRelativize(fwd, str, theta);
    //theta /= 2; //temporary

    //some of these +'s and -'s will need to be changed when the wheels are flipped around
    frontLeft.spin(directionType::fwd, maxClamp(fwd + str + theta, maxOutputPct), velocityUnits::pct); 

    backLeft.spin(directionType::fwd, maxClamp(fwd - str + theta, maxOutputPct), velocityUnits::pct);

    frontRight.spin(directionType::fwd, maxClamp(fwd - str - theta, maxOutputPct), velocityUnits::pct);

    backRight.spin(directionType::fwd, maxClamp(fwd + str - theta, maxOutputPct), velocityUnits::pct);

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
    theta *= 50;

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

void Drive::stop() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
}

double Drive::gpsHeadingRad() {
    double radHeading = (-1 * gps1.heading() * M_PI / 180);
    return radHeading += (radHeading < -1 * M_PI ? 2 * M_PI : 0);
}

double Drive::gpsAngleRad() {
    double radAngle = (-1 * gps1.rotation() * M_PI / 180);
    return radAngle += (radAngle < -1 * M_PI ? 2 * M_PI : 0);
}

void Drive::driveForward(double fwd) {
    fwd = maxClamp(fwd, maxOutputPct);
    frontLeft.spin(directionType::fwd, fwd, velocityUnits::pct);
    frontRight.spin(directionType::fwd, fwd, velocityUnits::pct);
    backLeft.spin(directionType::fwd, fwd, velocityUnits::pct);
    backRight.spin(directionType::fwd, fwd, velocityUnits::pct);
}

void Drive::adjustCCW(double speed) {
    frontLeft.spin(directionType::fwd, -speed, velocityUnits::pct);
    frontRight.spin(directionType::fwd, speed, velocityUnits::pct);
    backLeft.spin(directionType::fwd, -speed, velocityUnits::pct);
    backRight.spin(directionType::fwd, speed, velocityUnits::pct);
    // wait(0.5, sec);
    // stop();
}

// ========================================================= //
// BEYOND HERE IS A VERY EXPERIMENTAL PORT OF OLD ROBOT CODE //
// ========================================================= //

double Drive::getAngleErrorOLD(double target) {
    //double currAngle = toRadians(-1 * gps1.rotation());
    double currHeading = toRadians(-1 * gps1.heading()); //fmod(currAngle, 2*M_PI); // gets angle and then restricts it to a fixed range
    if (fabs(currHeading) > M_PI) { //converts a (0 to 2pi) value to a (-pi to pi) value
        currHeading += (2 * M_PI) * (currHeading > 0 ? -1 : 1);
    }
    double error = target - currHeading; //ccw positive

    if (error > M_PI) { //check smaller target
        double error2 = (target - 2*M_PI) - currHeading;
        if (fabs(error) > fabs(error2)) {
            error = error2;
        }
    } else if (error < -1*M_PI) { //check larger target
        double error2 = (target + 2*M_PI) - currHeading;
        if (fabs(error) > fabs(error2)) {
            error = error2;
        }
    }
    
    return error;
}

double Drive::getAngleError(double target) {
    double currHeading = toRadians(-1 * gps1.heading());
    double error = target - currHeading; //ccw positive

    if (fabs(error) > M_PI) { //converts a (0 to 2pi) value to a (-pi to pi) value
        error += (2*M_PI) * (error > 0 ? -1 : 1);
    }
    
    return error;
}

void Drive::turnPID(double targetHeading) {
    double error = 0;
    double errorLast;
    double lastTime = 0;
    double dt;
    double integrationStored = 0;
    pid_timer.reset();
    while(true) {
        activePID = true;
        errorLast = error;
        error = getAngleError(targetHeading);
        dt = pid_timer.time() - lastTime;

        double P_comp = a_P * error;
        double D_comp = 0;

        if (errorLast != 0) {
            double D_comp = a_D * (error - errorLast) / dt;
        }

        integrationStored += (error * dt);
        double I_comp = a_I * integrationStored;

        double output = P_comp + I_comp + D_comp;

        adjustCCW(output);

        if (fabs(error) < 0.02 && fabs((error-errorLast)/dt) < 0.02) {
            break;
        }
    }
    activePID = false;
    stop();
}
/*
void turnToHeading(double targetHeading) {
    double targetAngle = getTarget(targetHeading);
    while (gps.getTheta() < (targetAngle - 0.05) || gps.getTheta() > (targetAngle + 0.05)) {
        //drivePure(0, 0, (targetAngle > gps.getTheta()) ? 20 : -20);
        adjustRight(targetAngle > gps.getTheta() ? 20 : -20);
        //targetAngle = getTarget(targetHeading);
    }
    stop();
}*/

double Drive::getAngleToPoint(double x2, double y2) {
    double x1 = gps1.xPosition(vex::distanceUnits::mm);
    double y1 = gps1.yPosition(vex::distanceUnits::mm);

    double theta = atan2(y2 - y1, x2 - x1);

    return theta + M_PI / 2; //addition is because theta is 90 deg off????
}
  
// find error between target point, and current point
double Drive::getDistanceError(double targetX, double targetY, int count = 0) {

    // get current x, and y position
    double currentX = gps1.xPosition(vex::distanceUnits::mm);
    double currentY = gps1.yPosition(vex::distanceUnits::mm);

    // find the distance between the targets, and currents 
    double errorX = targetX - currentX;
    double errorY = targetY - currentY;

    // find the length of the hypotenuse
    double errorTan = sqrt(pow(errorX, 2) + pow(errorY, 2));

    // if(count % 100 == 0){
    //     cout << "first: " << errorTan << endl;
    // }

    // accounts for the angle always being positive by making it negative if the robot is facing away from the target point
    double currAngle = fmod(toRadians(gps1.rotation()), 2*M_PI);
    currAngle += currAngle < 0 ? 2*M_PI : 0;
    double angleDifference = fabs(getAngleToPoint(targetX, targetY) - currAngle);
    errorTan *= (angleDifference < M_PI/2 || angleDifference > 3*M_PI/2) ? 1 : -1;
    //errorTan *= abs(getAngleToPoint(targetX, targetY) - fmod(gps.getTheta(), 2*M_PI)) < M_PI/2 ? 1 : -1; // angle from where robot is facing to angle towards desired location
    
    // if(count % 100 == 0){
    //     cout << "second: " << errorTan << endl;
    // }
    return errorTan;
}

// function to go to a point using PID
void Drive::goToPointPID(double targetX, double targetY) {
    //double currentX = gps1.xPosition(vex::distanceUnits::mm);
    //double currentY = gps1.yPosition(vex::distanceUnits::mm);

    double error = 0;
    double errorLast;
    double dt;
    double lastTime = 0;
    double integrationStored = 0;
    pid_timer2.reset();

    //int count = 0;
  
    while(true){
        activePID = true;
        errorLast = error;
        error = getDistanceError(targetX, targetY);
        dt = pid_timer2.time() - lastTime;
      
        double P_comp = d_P * error;
        double D_comp = 0;

        if (errorLast != 0) {
            double D_comp = d_D * (error - errorLast) / dt;
        }

        integrationStored += (error * dt);
        double I_comp = d_I * integrationStored;

        double output = P_comp + I_comp + D_comp;

        driveForward(output);

        if (fabs(error) < 0.2 && fabs(error-errorLast) / dt) {
            break;
        }
    }
    activePID = false;
    stop();
}

void Drive::turnToPoint(double targetX, double targetY, bool flipped) {
    double theta = getAngleToPoint(targetX, targetY);
    if (flipped) {
        theta += M_PI * (theta <= 0 ? 1 : -1);
    }
    //theta += flipped ? -1*M_PI : 0; 
    turnPID(theta);
}

void Drive::turnAndDrivePID(double targetX, double targetY) {
    turnToPoint(targetX, targetY);
    goToPointPID(targetX, targetY);
}