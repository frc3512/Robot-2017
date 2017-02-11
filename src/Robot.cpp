// Copyright (c) FRC Team 3512, Spartatroniks 2017. All Rights Reserved.

#include "Robot.hpp"

using namespace std::chrono_literals;

Robot::Robot() {
    dsDisplay.AddAutoMethod("No-op", &Robot::AutoNoop, this);
    dsDisplay.AddAutoMethod("LeftGear", &Robot::AutoLeftGear, this);
}

void Robot::OperatorControl() {
    while (IsEnabled() && IsOperatorControl()) {
        if (driveStick1.GetTrigger()) {
            robotDrive.Drive(driveStick1.GetY() * 0.5, driveStick2.GetX() * 0.5,
                             driveStick2.GetRawButton(2));
        } else {
            robotDrive.Drive(driveStick1.GetY(), driveStick2.GetX(),
                             driveStick2.GetRawButton(2));
        }
        if (grabberStick.GetRawButton(4)) {
            robotGrabber.Set(0.5);
        } else if (grabberStick.GetRawButton(6)) {
            robotGrabber.Set(-0.5);
        } else {
            robotGrabber.Set(0);
        }

        if (grabberStick.GetTrigger()) {
            solenoidSwitch->Set(true);
        } else {
            solenoidSwitch->Set(false);
        }
        if (grabberStick.GetPOV() == 0) {
            robotWinch.Set(1.0);
        } else if (grabberStick.GetPOV() == 180) {
            robotWinch.Set(-1.0);
        } else {
            robotWinch.Set(0);
        }

        std::cout << "DIO 0 Backward Limit: "
                  << DigitalInputHandler::Get(0)->Get() << std::endl;
        std::cout << "DIO 1 Forward Limit: "
                  << DigitalInputHandler::Get(1)->Get() << std::endl;

        // GPU.GearPickup();
    }
}

void Robot::Autonomous() {
    autoTimer.Reset();
    autoTimer.Start();

    robotDrive.ResetEncoders();
    dsDisplay.ExecAutonomous();
}

void Robot::Disabled() {
    while (IsDisabled()) {
        DS_PrintOut();
        std::this_thread::sleep_for(10ms);
    }
}

void Robot::Test() {
    while (IsEnabled() && IsTest()) {
        DS_PrintOut();
        std::this_thread::sleep_for(10ms);
    }
}

void Robot::DS_PrintOut() {}

START_ROBOT_CLASS(Robot)
