// Copyright (c) FRC Team 3512, Spartatroniks 2016-2017. All Rights Reserved.

#include <cmath>

#include "../Robot.hpp"

using namespace std::chrono_literals;

/* Moves forwards a set distance and then stops with gear penetrated by
 * airship's divot*/
void Robot::AutoCenterGear() {
    robotDrive.StartClosedLoop();

    robotDrive.ResetEncoders();
    robotDrive.ResetGyro();
    shifter.Set(true);  // low gear
    robotDrive.SetPositionReference(114.3 - 39 /*robot length*/);
    robotDrive.SetAngleReference(0);
    gearPunch.Set(frc::DoubleSolenoid::kForward); // TODO: Add this to left and right gear

    while (IsAutonomous() && IsEnabled() &&
           std::abs(robotDrive.GetPosReference() - robotDrive.GetPosition()) >
               0.001) {
        DS_PrintOut();

        std::this_thread::sleep_for(10ms);
    }
    robotDrive.StopClosedLoop();

    robotDrive.Drive(0.0, 0.0, false);
}
