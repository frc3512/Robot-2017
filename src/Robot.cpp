// Copyright (c) 2017 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

Robot::Robot() {
    // Auton: does nothing
    dsDisplay.AddAutoMethod("No-op", [] {});

    dsDisplay.AddAutoMethod("LeftGear", std::bind(&Robot::AutoLeftGear, this));
    dsDisplay.AddAutoMethod("CenterGear",
                            std::bind(&Robot::AutoCenterGear, this));
    dsDisplay.AddAutoMethod("RightGear",
                            std::bind(&Robot::AutoRightGear, this));
    dsDisplay.AddAutoMethod("BaseLine", std::bind(&Robot::AutoBaseLine, this));

    server.SetSource(camera1);

    camera1.SetResolution(160, 120);
    camera1.SetFPS(15);
    // camera2.SetResolution(320, 240);
    // camera2.SetFPS(15);

    // liveGrapher.SetSendInterval(50ms);
}

void Robot::DisabledInit() { robotDrive.StopClosedLoop(); }

void Robot::AutonomousInit() {
    autoTimer.Reset();
    autoTimer.Start();
}

void Robot::TeleopInit() {
    robotDrive.StopClosedLoop();

    robotDrive.ResetEncoders();
    robotDrive.ResetGyro();
}

void Robot::TestInit() {
    arm.Set(DoubleSolenoid::kReverse);        // Raise arm
    gearPunch.Set(DoubleSolenoid::kForward);  // Extends gear punch
}

void Robot::RobotPeriodic() { DS_PrintOut(); }

void Robot::DisabledPeriodic() {
    if (grabberStick.GetRawButtonPressed(12)) {
        robotDrive.CalibrateGyro();
    }
}

void Robot::AutonomousPeriodic() {
    // AutoCenterGear();
    // AutoRightGear();
    dsDisplay.ExecAutonomous();
}

void Robot::TeleopPeriodic() {
    // Drive Stick Controls
    if (driveStick1.GetRawButton(1)) {
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

    if (driveStick2.GetRawButtonPressed(1)) {
        shifter.Set(!shifter.Get());
    }

    // Appendage Stick Controls

    if (grabberStick.GetRawButtonPressed(1)) {
        claw.Set(!claw.Get());
    }

    if (grabberStick.GetRawButtonPressed(3)) {
        arm.Set(frc::DoubleSolenoid::kForward);
    }
    if (grabberStick.GetRawButtonPressed(5)) {
        arm.Set(frc::DoubleSolenoid::kReverse);
    }

    if (grabberStick.GetRawButtonPressed(4)) {
        gearPunch.Set(frc::DoubleSolenoid::kReverse);
    }
    if (grabberStick.GetRawButtonPressed(6)) {
        gearPunch.Set(frc::DoubleSolenoid::kForward);
    }

    if (grabberStick.GetPOV() == 0) {
        robotWinch.Set(1);
    } else if (grabberStick.GetPOV() == 180) {
        robotWinch.Set(-1);
    } else {
        robotWinch.Set(0);
    }

    // Camera

    /*if (grabberStick.GetRawButtonPressed(11)) {
        if (server.GetSource() == camera1) {
            server.SetSource(camera2);
        } else {
            server.SetSource(camera1);
        }
    }*/
}

void Robot::DS_PrintOut() {
    // if (liveGrapher.HasIntervalPassed()) {
    // liveGrapher.GraphData(robotDrive.GetAngle(), "Gyro Angle");
    // liveGrapher.GraphData(robotDrive.GetRate(), "Gyro Rate");
    // liveGrapher.GraphData((300 * driveStick2.GetX()), "Gyro Rate Ref");
    // liveGrapher.GraphData(-robotDrive.GetRightRate(), "Encoder Right
    // Rate");
    // liveGrapher.GraphData(robotDrive.GetLeftRate(), "Encoder Left Rate");

    // liveGrapher.GraphData(robotDrive.GetFilteredRate(), "Filtered Gyro");

    // liveGrapher.GraphData(robotDrive.GetPosition(), "Position");
    // liveGrapher.GraphData(93.3 * -driveStick1.GetY(), "Position Ref");
    // liveGrapher.GraphData(robotDrive.GetAngle(), "Angle");
    // liveGrapher.GraphData(/*30 * driveStick2.GetX()*/ 0, "Angle Ref");

    // liveGrapher.ResetInterval();
    //}

    robotDrive.Debug();
    dsDisplay.ReceiveFromDS();
}

START_ROBOT_CLASS(Robot)
