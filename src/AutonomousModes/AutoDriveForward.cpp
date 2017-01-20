// Copyright (c) FRC Team 3512, Spartatroniks 2016-2017. All Rights Reserved.

#include "../DigitalInputHandler.hpp"
#include "../Robot.hpp"

using namespace std::chrono_literals;

// Drives forward
void Robot::AutoDriveForward() {
    Timer timer;
    timer.Start();

    while (!timer.HasPeriodPassed(10.0) && IsAutonomous() && IsEnabled()) {
        DS_PrintOut();
        if (timer.Get() < 1.0 ){

        }
        std::this_thread::sleep_for(10ms);
    }
    while (!timer.HasPeriodPassed(1.50) && IsAutonomous() && IsEnabled()) {
        DS_PrintOut();
        robotDrive.Drive(1, 0, false);

        std::this_thread::sleep_for(10ms);
    }

    robotDrive.Drive(0.0, 0.0, false);

    while (IsAutonomous() && IsEnabled()) {
        DS_PrintOut();
        std::this_thread::sleep_for(10ms);
    }
}
