// Copyright (c) 2017-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <cameraserver/CameraServer.h>
#include <frc2/Timer.h>

#include <cscore.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Joystick.h>
#include <frc/Solenoid.h>
#include <frc/TimedRobot.h>

#include "Constants.hpp"
#include "dsdisplay/DSDisplay.hpp"
#include "subsystems/CANTalonGroup.hpp"
#include "subsystems/Drivetrain.hpp"

class Robot : public frc::TimedRobot {
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
    using WPI_TalonSRX = ctre::phoenix::motorcontrol::can::WPI_TalonSRX;

    Drivetrain robotDrive;
    frc::Solenoid claw{0};
    frc::DoubleSolenoid arm{1, 2};
    frc::DoubleSolenoid gearPunch{3, 4};

    frc::Solenoid shifter{5};

    WPI_TalonSRX grabberMotor{15};
    CANTalonGroup robotGrabber{grabberMotor};

    WPI_TalonSRX winchMotor{3};
    CANTalonGroup robotWinch{winchMotor};

    frc::Joystick driveStick1{k_driveStick1Port};
    frc::Joystick driveStick2{k_driveStick2Port};
    frc::Joystick grabberStick{k_grabberStickPort};

    frc2::Timer autoTimer;

    // Used for sending data to the Driver Station
    DSDisplay dsDisplay{k_dsPort};

    // Camera
    cs::UsbCamera camera1{"Camera 1", 0};
    // cs::UsbCamera camera2{"Camera 2", 1};

    cs::MjpegServer server{"Server", k_mjpegServerPort};
};
