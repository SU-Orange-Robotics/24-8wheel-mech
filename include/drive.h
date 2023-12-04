#include "vex.h"
#include <math.h>

#pragma once

class Drive {
        
    private:
        const double deadzone = 10; // from 0 to 100
        const double maxOutputPct = 90; // limits maximum motor output percentage

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

        void inputAdjust(double &fwd, double &str, double &theta);

        void fieldRelativize(double &fwd, double &str, double &theta);


        bool activePID;

        const double a_P = 50;  //50
        const double a_I = 0;   //0
        const double a_D = 0.3; //0.1
        vex::timer pid_timer;

        const double d_P = 1.0;
        const double d_D = 3.0;
        const double d_I = 0.00;
        vex::timer pid_timer2;

        double getAngleErrorOLD(double target);

        double getAngleError(double target);

        

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

        double gpsHeadingRad();

        double gpsAngleRad();


        // make this private at some point, it is only public for debugging purposes on the controller display
        double getAngleToPoint(double x2, double y2);



        void driveForward(double fwd);

        void adjustCCW(double speed);

        void turnPID(double targetHeading);

        void goToPointPID(double targetX, double targetY);

        void turnToPoint(double targetX, double targetY, bool flipped = false);

        void turnAndDrivePID(double targetX, double targetY);

        bool pidActive() {
            return activePID;
        }

};
