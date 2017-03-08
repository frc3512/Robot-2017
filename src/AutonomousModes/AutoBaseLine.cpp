// Copyright (c) FRC Team 3512, Spartatroniks 2016-2017. All Rights Reserved.

#include <cmath>

#include "../Robot.hpp"

using namespace std::chrono_literals;

/* Drives forward until passing white line 120 inches away from start */
void Robot::AutoBaseLine() {
    robotDrive.StartClosedLoop();

    robotDrive.ResetEncoders();
    robotDrive.ResetGyro();
    shifter.Set(true);  // low gear
    robotDrive.SetPositionReference(39 + 120 + 10); // robot length + 120 + safety inches = 169 in
    robotDrive.SetAngleReference(0);

    while (IsAutonomous() && IsEnabled() &&
           std::abs(robotDrive.GetPosReference() - robotDrive.GetPosition()) >
               0.001) { // TODO: replace with drivetrain function
        DS_PrintOut();

        std::this_thread::sleep_for(10ms);
    }
    robotDrive.StopClosedLoop();

    robotDrive.Drive(0.0, 0.0, false);
}
