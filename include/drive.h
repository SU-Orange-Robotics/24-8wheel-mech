#include "vex.h"
#include <math.h>

#pragma once

class Drive {
        
    private:
        const double deadzone = 10; // from 0 to 100
        const double maxOutputPct = 75; // limits maximum motor output percentage

        double originHeading;
        double originCorr;
        bool invertDrive;
        bool relativeDrive;

        static double toRadians(double deg) {
            return (deg * M_PI / 180);
        }

        static double toDegrees(double rad) {
            return (rad * 180 / M_PI);
        }

        static double maxClamp(double input, double max) {
            return (fabs(input) <= max ? input : max * (input / fabs(input)));
        }

        static double headingRad() {
            return toRadians(gps1.heading());
        }

        void inputAdjust(double &fwd, double &str, double &theta);

        void fieldRelativize(double &fwd, double &str, double &theta);


        bool activePID;

        const double a_P = 50;  //50
        const double a_I = 0;   //0
        const double a_D = 0.2; //0.1
        vex::timer pid_timer;

        const double d_P = 3.0;
        const double d_D = 3.0;
        const double d_I = 0.00;
        vex::timer pid_timer2;

        double getAngleErrorOLD(double target);

        double getAngleError(double target);

        double getAngleToPoint(double x2, double y2);

        double getDistanceError(double targetX, double targetY, int count);

    public:
        Drive();

        void drive(double y, double x, double theta);

        void toggleInvertedDrive() {
            invertDrive = invertDrive ? false : true;
        }

        void toggleRelativeDrive() {
            relativeDrive = relativeDrive ? false : true;
        }

        void resetHeading();

        void stop();

        void driveForward(double fwd);

        void adjustRight(double speed);

        void turnPID(double targetHeading);

        void goToPointPID(double targetX, double targetY);

        void turnToPoint(double targetX, double targetY, bool flipped = false);

        void turnAndDrivePID(double targetX, double targetY);

};
