// Copyright (c) FRC Team 3512, Spartatroniks 2016-2017. All Rights Reserved.

#pragma once

#include <CameraServer.h>
#include <Joystick.h>
#include <SampleRobot.h>

#include "ButtonTracker.hpp"
#include "Constants.hpp"
#include "DSDisplay.hpp"
#include "LiveGrapher/GraphHost.hpp"
#include "Subsystems/DriveTrain.hpp"

/**
 * Implements the main robot class
 */
class Robot : public SampleRobot {
public:
    Robot();
    void OperatorControl();
    void Autonomous();
    void Disabled();
    void Test();

    void AutoNoop();
    void AutoMotionProfile();
    void AutoDriveForward();


    void DS_PrintOut();

private:
    DriveTrain robotDrive;
    Arm arm;

    frc::Joystick driveStick1{k_driveStick1Port};
    frc::Joystick driveStick2{k_driveStick2Port};

    ButtonTracker shootButtons{k_shootStickPort};

    frc::Timer autoTimer;
    frc::Timer displayTimer;

    // Used for sending data to the Driver Station
    DSDisplay& dsDisplay{DSDisplay::GetInstance(k_dsPort)};

    // The LiveGrapher host
    GraphHost pidGraph{3513};

    // Camera
    // frc::CameraServer* camera = frc::CameraServer::GetInstance();
};
