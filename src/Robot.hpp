// Copyright (c) FRC Team 3512, Spartatroniks 2017. All Rights Reserved.

#pragma once

#include <ADXRS450_Gyro.h>
#include <CANTalon.h>
#include <Joystick.h>
#include <SampleRobot.h>
#include <Timer.h>

#include "Constants.hpp"
#include "DSDisplay.hpp"
#include "LiveGrapher/GraphHost.hpp"
#include "Subsystems/DriveTrain.hpp"
#include "Subsystems/GearPickup.hpp"

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

    void DS_PrintOut();

private:
    DriveTrain robotDrive;
    ADXRS450_Gyro robotGyro;
    GearPickup GPU;

    GearBox robotWinch{-1, -1, -1, 3};

    frc::Joystick driveStick1{k_driveStick1Port};
    frc::Joystick driveStick2{k_driveStick2Port};
    frc::Joystick grabberStick{k_grabberStickPort};

    frc::Timer autoTimer;
    frc::Timer displayTimer;

    // Used for sending data to the Driver Station
    DSDisplay& dsDisplay{DSDisplay::GetInstance(k_dsPort)};

    // The LiveGrapher host
    GraphHost pidGraph{3513};
};
