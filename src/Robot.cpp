// Copyright (c) FRC Team 3512, Spartatroniks 2017. All Rights Reserved.

#include "Robot.hpp"

using namespace std::chrono_literals;

Robot::Robot() {
    dsDisplay.AddAutoMethod("No-op", &Robot::AutoNoop, this);
    dsDisplay.AddAutoMethod("LeftGear", &Robot::AutoLeftGear, this);
    dsDisplay.AddAutoMethod("CenterGear", &Robot::AutoCenterGear, this);
    dsDisplay.AddAutoMethod("RightGear", &Robot::AutoRightGear, this);
    dsDisplay.AddAutoMethod("BaseLine", &Robot::AutoBaseLine, this);

    server.SetSource(camera1);

    liveGrapher.SetSendInterval(50ms);

    robotGrabber.SetLimitOnHigh(false);
}

void Robot::OperatorControl() {
    robotDrive.StopClosedLoop();

    robotDrive.ResetEncoders();
    robotDrive.ResetGyro();
    while (IsEnabled() && IsOperatorControl()) {
        // Drive Stick Controls
        if (driveStick1.GetTrigger()) {
            robotDrive.Drive(driveStick1.GetY() * 0.5, driveStick2.GetX() * 0.5,
                             driveStick2.GetRawButton(2));
        } else {
            robotDrive.Drive(driveStick1.GetY(), driveStick2.GetX(),
                             driveStick2.GetRawButton(2));
        }

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

        std::this_thread::sleep_for(10ms);
    }
}

void Robot::Autonomous() {
    autoTimer.Reset();
    autoTimer.Start();

    AutoBaseLine();

    // dsDisplay.ExecAutonomous();

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
    if (liveGrapher.HasIntervalPassed()) {
        // liveGrapher.GraphData(robotDrive.GetAngle(), "Gyro Angle");
        // liveGrapher.GraphData(robotDrive.GetRate(), "Gyro Rate");
        // liveGrapher.GraphData((300 * driveStick2.GetX()), "Gyro Rate Ref");
        // liveGrapher.GraphData(-robotDrive.GetRightRate(), "Encoder Right Rate");
        // liveGrapher.GraphData(robotDrive.GetLeftRate(), "Encoder Left Rate");

        // liveGrapher.GraphData(robotDrive.GetFilteredRate(), "Filtered Gyro");

        liveGrapher.GraphData(robotDrive.GetPosition(), "Position");
        liveGrapher.GraphData(93.3 * -driveStick1.GetY(), "Position Ref");
        liveGrapher.GraphData(robotDrive.GetAngle(), "Angle");
        liveGrapher.GraphData(/*30 * driveStick2.GetX()*/ 0, "Angle Ref");

        liveGrapher.ResetInterval();
    }

    dsDisplay.ReceiveFromDS();
}

START_ROBOT_CLASS(Robot)
