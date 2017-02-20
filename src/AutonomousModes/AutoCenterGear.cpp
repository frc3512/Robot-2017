// Copyright (c) FRC Team 3512, Spartatroniks 2016-2017. All Rights Reserved.

#include "../Robot.hpp"

using namespace std::chrono_literals;

/* Moves forwards a set distance and then stops with gear penetrated by
 * airship's divot*/
void Robot::AutoCenterGear() {
	shifter.Set(true);  // low gear
    while (IsAutonomous() && IsEnabled() && 1 /*not at setpoint*/) {
        DS_PrintOut();
        robotDrive.Drive(0.75, 0.0, false);
        std::this_thread::sleep_for(10ms);
    }
    robotDrive.Drive(0.0, 0.0, false);
}
