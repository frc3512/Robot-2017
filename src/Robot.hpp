// Copyright (c) FRC Team 3512, Spartatroniks 2017. All Rights Reserved.

#pragma once

#include <cscore.h>

#include <CANTalon.h>
#include <CameraServer.h>
#include <DoubleSolenoid.h>
#include <Joystick.h>
#include <SampleRobot.h>
#include <Solenoid.h>
#include <Timer.h>

#include "ButtonTracker.hpp"
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

    void AutoLeftGear();
    void AutoCenterGear();
    void AutoRightGear();
    void AutoBaseLine();

    void DS_PrintOut();

private:
    DriveTrain robotDrive;
    Solenoid claw{0};
    DoubleSolenoid arm{1, 2};
    DoubleSolenoid gearPunch{3, 4};

    Solenoid shifter{5};

    GearBox robotGrabber{-1, 1, 0, 15};
    GearBox robotWinch{-1, -1, -1, 3};

    frc::Joystick driveStick1{k_driveStick1Port};
    frc::Joystick driveStick2{k_driveStick2Port};
    frc::Joystick grabberStick{k_grabberStickPort};

    ButtonTracker armButtons{k_grabberStickPort};
    ButtonTracker drive2Buttons{k_driveStick2Port};

    frc::Timer autoTimer;

    // Used for sending data to the Driver Station
    DSDisplay& dsDisplay{DSDisplay::GetInstance(k_dsPort)};

    // LiveGrapher host
    // GraphHost liveGrapher{k_liveGrapherPort};

    // Camera
    cs::UsbCamera camera1{"Camera 1", 0};
    // cs::UsbCamera camera2{"Camera 2", 1};

    cs::MjpegServer server{"Server", k_mjpegServerPort};
};
