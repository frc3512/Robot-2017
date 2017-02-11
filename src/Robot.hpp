// Copyright (c) FRC Team 3512, Spartatroniks 2017. All Rights Reserved.

#pragma once

#include <ADXRS450_Gyro.h>
#include <CameraServer.h>
#include <Joystick.h>
#include <SampleRobot.h>
#include <Timer.h>

#include "Constants.hpp"
#include "DSDisplay.hpp"
#include "LiveGrapher/GraphHost.hpp"
#include "Subsystems/DriveTrain.hpp"
#include "Subsystems/Grabber.hpp"
#include "Subsystems/Winch.hpp"

class Robot : public SampleRobot {
public:
    Robot();
    virtual ~Robot() = default;

    void OperatorControl();
    void Autonomous();
    void Disabled();
    void Test();

    void AutoNoop();
    void AutoLeftGear();
    void AutoCenterGear();
    void AutoRightGear();

    void DS_PrintOut();

private:
    DriveTrain robotDrive;
    ADXRS450_Gyro robotGyro;

    frc::Joystick driveStick1{k_driveStick1Port};
    frc::Joystick driveStick2{k_driveStick2Port};
    frc::Joystick grabberStick{k_grabber};

    frc::Timer autoTimer;
    frc::Timer displayTimer;

    // Used for sending data to the Driver Station
    DSDisplay& dsDisplay{DSDisplay::GetInstance(k_dsPort)};

    // The LiveGrapher host
    GraphHost pidGraph{3513};

    frc::CameraServer* camera = frc::CameraServer::GetInstance();
};
