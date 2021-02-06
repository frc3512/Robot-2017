// Copyright (c) 2017-2021 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

Robot::Robot() {
    m_autonChooser.AddAutonomous("LeftGear", [=] { AutoLeftGear(); });
    m_autonChooser.AddAutonomous("CenterGear", [=] { AutoCenterGear(); });
    m_autonChooser.AddAutonomous("RightGear", [=] { AutoRightGear(); });
    m_autonChooser.AddAutonomous("BaseLine", [=] { AutoBaseLine(); });

    server.SetSource(camera1);

    camera1.SetResolution(160, 120);
    camera1.SetFPS(15);
    // camera2.SetResolution(320, 240);
    // camera2.SetFPS(15);
}

void Robot::DisabledInit() {
    m_autonChooser.EndAutonomous();
    robotDrive.StopClosedLoop();
}

void Robot::TeleopInit() {
    m_autonChooser.EndAutonomous();
    robotDrive.StopClosedLoop();

    robotDrive.ResetEncoders();
    robotDrive.ResetGyro();
}

void Robot::TestInit() {
    m_autonChooser.EndAutonomous();

    arm.Set(frc::DoubleSolenoid::kReverse);        // Raise arm
    gearPunch.Set(frc::DoubleSolenoid::kForward);  // Extends gear punch
}

void Robot::RobotPeriodic() { DS_PrintOut(); }

void Robot::DisabledPeriodic() {
    if (grabberStick.GetRawButtonPressed(12)) {
        robotDrive.CalibrateGyro();
    }
}

void Robot::AutonomousPeriodic() { m_autonChooser.AwaitRunAutonomous(); }

void Robot::TeleopPeriodic() {
    // Drive Stick Controls
    if (driveStick1.GetRawButton(1)) {
        robotDrive.Drive(driveStick1.GetY() * 0.5, driveStick2.GetX() * 0.5,
                         driveStick2.GetRawButton(2));
    } else {
        robotDrive.Drive(driveStick1.GetY(), driveStick2.GetX(),
                         driveStick2.GetRawButton(2));
    }

    if (grabberStick.GetRawButton(4) && !forwardGrabberLimit.Get()) {
        grabberMotor.Set(1.0);
    } else if (grabberStick.GetRawButton(6) && !reverseGrabberLimit.Get()) {
        grabberMotor.Set(-1.0);
    } else {
        grabberMotor.Set(0.0);
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
        winchMotor.Set(1.0);
    } else if (grabberStick.GetPOV() == 180) {
        winchMotor.Set(-1.0);
    } else {
        winchMotor.Set(0.0);
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

void Robot::DS_PrintOut() { robotDrive.Debug(); }

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif  // RUNNING_FRC_TESTS
