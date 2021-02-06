// Copyright (c) 2017-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <cameraserver/CameraServer.h>

#include <cscore.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <frc/DigitalInput.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Joystick.h>
#include <frc/Solenoid.h>
#include <frc/TimedRobot.h>

#include "AutonomousChooser.hpp"
#include "Constants.hpp"
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
    frc::DigitalInput forwardGrabberLimit{1};
    frc::DigitalInput reverseGrabberLimit{0};

    WPI_TalonSRX winchMotor{3};

    frc::Joystick driveStick1{kDriveStick1Port};
    frc::Joystick driveStick2{kDriveStick2Port};
    frc::Joystick grabberStick{kGrabberStickPort};

    frc3512::AutonomousChooser m_autonChooser{"No-op", [] {}};

    // Camera
    cs::UsbCamera camera1{"Camera 1", 0};
    // cs::UsbCamera camera2{"Camera 2", 1};

    cs::MjpegServer server{"Server", kMjpegServerPort};
};
