// Copyright (c) FRC Team 3512, Spartatroniks 2016-2017. All Rights Reserved.

#include "Robot.hpp"

#include <chrono>

using namespace std::chrono_literals;

#include "Utility.hpp"

Robot::Robot() {
    dsDisplay.AddAutoMethod("No-op", &Robot::AutoNoop, this);
    dsDisplay.AddAutoMethod("Drive Forward", &Robot::AutoDriveForward, this);

    // camera->StartAutomaticCapture();

    pidGraph.SetSendInterval(5ms);

    displayTimer.Start();
}

void Robot::OperatorControl() {
    while (IsEnabled() && IsOperatorControl()) {
        // Enables QuickTurn if button is pressed
        // If trigger is pressed, move at half speed
        if (driveStick1.GetTrigger()) {
            robotDrive.Drive(-driveStick1.GetY() * 0.5,
                             driveStick2.GetX() * 0.5,
                             driveStick2.GetRawButton(2));
        } else {
            robotDrive.Drive(-driveStick1.GetY(), driveStick2.GetX(),
                             driveStick2.GetRawButton(2));
        }

        // Could be used if same winch design from 2016
        if (armStick.GetPOV() == 0) {
            arm.SetManualWinchHeight(1);
        } else if (armStick.GetPOV() == 180) {
            arm.SetManualWinchHeight(-1);
        } else {
            arm.SetManualWinchHeight(0);
        }

        shootButtons.Update();

        shooter.UpdateState();

        DS_PrintOut();

        std::this_thread::sleep_for(10ms);
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
        shooter.UpdateState();
        DS_PrintOut();
        std::this_thread::sleep_for(10ms);
    }

    robotDrive.ReloadPID();
}

void Robot::Test() {
    shooter.SetManualOverride(false);
    while (IsEnabled() && IsTest()) {
        if (DigitalInputHandler::Get(k_leftArmBottomLimitChannel)->Get()) {
            arm.SetArmHeight(0.1);
        } else {
            arm.SetArmHeight(0.0);
        }

        DS_PrintOut();
        std::this_thread::sleep_for(10ms);
    }
}

void Robot::DS_PrintOut() {
    if (pidGraph.HasIntervalPassed()) {
        pidGraph.GraphData(shooter.GetShooterHeightSetpoint().displacement,
                           "ShtHei SP");
        pidGraph.GraphData(shooter.GetShooterHeight(), "Sht Height POS");
        pidGraph.GraphData(shooter.m_shooterHeightGrbx.GetSpeed(),
                           "Sht Hght Spd");
        // pidGraph.GraphData(robotDrive.GetLeftDisplacement(), "Left PV (DR)");
        // pidGraph.GraphData(robotDrive.GetRightDisplacement(), "Right PV
        // (DR)");
        // pidGraph.GraphData(robotDrive.DiffPIDGet(), "Diff PID (DR)");

        pidGraph.ResetInterval();
    }

    if (displayTimer.HasPeriodPassed(0.5)) {
        // Send things to DS display
        dsDisplay.Clear();

        dsDisplay.AddData("ENCODER_LEFT", robotDrive.GetLeftDisplacement());
        dsDisplay.AddData("ENCODER_RIGHT", robotDrive.GetRightDisplacement());

        dsDisplay.SendToDS();
    }
    dsDisplay.ReceiveFromDS();

    std::cout << " Shooter Angle: " << shooter.GetShooterHeight() << std::endl;
    std::cout << " limit: "
              << DigitalInputHandler::Get(k_leftArmBottomLimitChannel)->Get()
              << std::endl;
}

START_ROBOT_CLASS(Robot);
