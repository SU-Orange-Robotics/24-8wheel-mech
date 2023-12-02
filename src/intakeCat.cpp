#include "vex.h"
#include "robot-config.h"
#include "intakeCat.h"

using namespace vex;

void intakeSpin(bool reversed) {
    intake.spin(directionType::fwd, intakePow * (reversed ? -1 : 1), percentUnits::pct);
}

void intakeStop() {
    intake.stop();
}

void catapultArm() {
    catapultA.spin(directionType::fwd, catPow, percentUnits::pct);
    catapultB.spin(directionType::fwd, catPow, percentUnits::pct);
    catapultC.spin(directionType::fwd, catPow, percentUnits::pct);
}

void catapultStop() {
    catapultA.stop();
    catapultB.stop();
    catapultC.stop();
}