// Copyright (c) FRC Team 3512, Spartatroniks 2017. All Rights Reserved.

#pragma once

#include <CANTalon.h>
#include <Compressor.h>
#include <Joystick.h>
#include <SampleRobot.h>
#include <Solenoid.h>
#include <Timer.h>

#include "Constants.hpp"
#include "DSDisplay.hpp"
#include "DigitalInputHandler.hpp"
#include "LiveGrapher/GraphHost.hpp"
#include "Subsystems/DriveTrain.hpp"

class Robot : public SampleRobot {
public:
    Robot();
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
    Compressor robotCompressor;
    Solenoid solenoidSwitch{0};

    Solenoid shifter{5};

    GearBox robotGrabber{-1, 1, 0, 15};
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
