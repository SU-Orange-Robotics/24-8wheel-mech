#include "vex.h"
#include <math.h>

#pragma once

class Drive {
        
    private:
        const double deadzone = 0.1;
        const double rcwScale = 0.5;

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

        void inputAdjust(double &fwd, double &str, double &theta);

        void fieldRelativize(double &fwd, double &str, double &theta);

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

};
