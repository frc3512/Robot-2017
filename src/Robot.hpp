// Copyright (c) 2017 FRC Team 3512. All Rights Reserved.

#pragma once

#include <CameraServer.h>
#include <DoubleSolenoid.h>
#include <Joystick.h>
#include <Solenoid.h>
#include <TimedRobot.h>
#include <Timer.h>
#include <cscore.h>
#include <ctrlib/CANTalon.h>

#include "Constants.hpp"
#include "DSDisplay/DSDisplay.hpp"
#include "LiveGrapher/GraphHost.hpp"
#include "Subsystems/CANTalonGroup.hpp"
#include "Subsystems/DriveTrain.hpp"

class Robot : public TimedRobot {
public:
    Robot();

    void DisabledInit() override;
    void AutonomousInit() override;
    void TeleopInit() override;
    void TestInit() override;

    void RobotPeriodic() override;
    void DisabledPeriodic() override;
    void AutonomousPeriodic() override;
    void TeleopPeriodic() override;

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

    CANTalon grabberMotor{15};
    CANTalonGroup robotGrabber{1, 0, grabberMotor};

    CANTalon winchMotor{3};
    CANTalonGroup robotWinch{winchMotor};

    frc::Joystick driveStick1{k_driveStick1Port};
    frc::Joystick driveStick2{k_driveStick2Port};
    frc::Joystick grabberStick{k_grabberStickPort};

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
