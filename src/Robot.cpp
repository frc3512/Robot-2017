// Copyright (c) FRC Team 3512, Spartatroniks 2017. All Rights Reserved.

#include "Robot.hpp"

using namespace std::chrono_literals;

// #define DRIVING

Robot::Robot() {
    dsDisplay.AddAutoMethod("No-op", &Robot::AutoNoop, this);
    dsDisplay.AddAutoMethod("LeftGear", &Robot::AutoLeftGear, this);
    dsDisplay.AddAutoMethod("CenterGear", &Robot::AutoCenterGear, this);
    dsDisplay.AddAutoMethod("RightGear", &Robot::AutoRightGear, this);

    server.SetSource(camera1);

    pidGraph.SetSendInterval(50ms);

    robotGrabber.SetLimitOnHigh(false);
}

void Robot::OperatorControl() {
#ifdef DRIVING
    robotDrive.StopClosedLoop();
#else
    robotDrive.StartClosedLoop();
#endif
    robotDrive.ResetEncoders();
    robotDrive.ResetGyro();
    while (IsEnabled() && IsOperatorControl()) {
#ifdef DRIVING
        // Drive Stick Controls
        if (driveStick1.GetTrigger()) {
            robotDrive.Drive(driveStick1.GetY() * 0.5, driveStick2.GetX() * 0.5,
                             driveStick2.GetRawButton(2));
        } else {
            robotDrive.Drive(driveStick1.GetY(), driveStick2.GetX(),
                             driveStick2.GetRawButton(2));
        }
#else
        robotDrive.SetAngleReference(/*30 * driveStick2.GetX()*/ 0);
        robotDrive.SetPositionReference(93.3 * -driveStick1.GetY());
#endif

        if (grabberStick.GetRawButton(4)) {
            robotGrabber.Set(1);
        } else if (grabberStick.GetRawButton(6)) {
            robotGrabber.Set(-1);
        }

        if (drive2Buttons.PressedButton(1)) {
            shifter.Set(!shifter.Get());
        }

        // Appendage Stick Controls

        if (armButtons.PressedButton(1)) {
            claw.Set(!claw.Get());
        }

        if (grabberStick.GetRawButton(3)) {
            arm.Set(frc::DoubleSolenoid::kReverse);
        }
        if (grabberStick.GetRawButton(5)) {
            arm.Set(frc::DoubleSolenoid::kForward);
        }

        if (grabberStick.GetRawButton(4)) {
            gearPunch.Set(frc::DoubleSolenoid::kReverse);
        }
        if (grabberStick.GetRawButton(6)) {
            gearPunch.Set(frc::DoubleSolenoid::kForward);
        }

        if (grabberStick.GetPOV() == 0) {
            robotWinch.Set(1);
        } else if (grabberStick.GetPOV() == 180) {
            robotWinch.Set(-1);
        } else {
            robotWinch.Set(0);
        }

        drive2Buttons.Update();
        armButtons.Update();
        robotDrive.Debug();

        DS_PrintOut();
    }
}

void Robot::Autonomous() {
    autoTimer.Reset();
    autoTimer.Start();

    dsDisplay.ExecAutonomous();

    DS_PrintOut();
}

void Robot::Disabled() {
    robotDrive.CalibrateGyro();
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

void Robot::DS_PrintOut() {
    if (pidGraph.HasIntervalPassed()) {
        // pidGraph.GraphData(robotDrive.GetAngle(), "Gyro Angle");
        // pidGraph.GraphData(robotDrive.GetRate(), "Gyro Rate");
        // pidGraph.GraphData((300 * driveStick2.GetX()), "Gyro Rate Ref");
        // pidGraph.GraphData(-robotDrive.GetRightRate(), "Encoder Right Rate");
        // pidGraph.GraphData(robotDrive.GetLeftRate(), "Encoder Left Rate");

        // pidGraph.GraphData(robotDrive.GetFilteredRate(), "Filtered Gyro");

        pidGraph.GraphData(robotDrive.GetPosition(), "Position");
        pidGraph.GraphData(93.3 * -driveStick1.GetY(), "Position Ref");
        pidGraph.GraphData(robotDrive.GetAngle(), "Angle");
        pidGraph.GraphData(/*30 * driveStick2.GetX()*/ 0, "Angle Ref");

        pidGraph.ResetInterval();
    }
}

START_ROBOT_CLASS(Robot)
