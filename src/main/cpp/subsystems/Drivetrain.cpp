// Copyright (c) 2016-2021 FRC Team 3512. All Rights Reserved.

#include "subsystems/Drivetrain.hpp"

#include <cmath>
#include <iostream>
#include <limits>

Drivetrain::Drivetrain() {
    m_drive.SetDeadband(kJoystickDeadband);

    m_leftFront.SetInverted(true);
    m_leftRear.SetInverted(true);
    m_rightFront.SetInverted(true);
    m_rightRear.SetInverted(true);

    m_leftGrbx.Set(0.0);
    m_rightGrbx.Set(0.0);

    m_controller.GetPositionPID().SetPID(kPosP, kPosI, kPosD);
    m_controller.GetAnglePID().SetPID(kAngleP, kAngleI, kAngleD);

    m_controller.GetPositionPID().SetOutputRange(-0.25, 0.25);
    m_controller.GetAnglePID().SetOutputRange(-0.5, 0.5);

    m_controller.SetPositionTolerance(1.5,
                                      std::numeric_limits<double>::infinity());
    m_controller.SetAngleTolerance(1.5,
                                   std::numeric_limits<double>::infinity());
}

int32_t Drivetrain::GetLeftRaw() const { return m_leftGrbx.Get(); }

int32_t Drivetrain::GetRightRaw() const { return m_rightGrbx.Get(); }

void Drivetrain::Drive(double throttle, double turn, bool isQuickTurn) {
    m_drive.CurvatureDrive(throttle, -turn, isQuickTurn);
}

void Drivetrain::ResetEncoders() {
    m_leftEncoder.Reset();
    m_rightEncoder.Reset();
}

void Drivetrain::SetLeftManual(double value) { m_leftGrbx.Set(value); }

void Drivetrain::SetRightManual(double value) { m_rightGrbx.Set(value); }

double Drivetrain::GetLeftDisplacement() { return m_leftEncoder.GetDistance(); }

double Drivetrain::GetRightDisplacement() {
    return m_rightEncoder.GetDistance();
}

double Drivetrain::GetLeftRate() { return m_leftEncoder.GetRate(); }

double Drivetrain::GetRightRate() { return m_rightEncoder.GetRate(); }

double Drivetrain::GetPosition() { return m_controller.GetPosition(); }

double Drivetrain::GetAngle() { return m_controller.GetAngle(); }

double Drivetrain::GetAngularRate() const { return m_gyro.GetRate(); }

void Drivetrain::StartClosedLoop() { m_controller.Enable(); }

void Drivetrain::StopClosedLoop() { m_controller.Disable(); }

void Drivetrain::SetPositionReference(double position) {
    m_posRef.Set(position);
}

void Drivetrain::SetAngleReference(double angle) { m_angleRef.Set(angle); }

double Drivetrain::GetPosReference() const { return m_posRef.GetOutput(); }

double Drivetrain::GetAngleReference() const { return m_angleRef.GetOutput(); }

bool Drivetrain::PosAtReference() const { return m_controller.AtPosition(); }

bool Drivetrain::AngleAtReference() const { return m_controller.AtAngle(); }

void Drivetrain::ResetGyro() { m_gyro.Reset(); }

void Drivetrain::CalibrateGyro() { m_gyro.Calibrate(); }

void Drivetrain::Debug() {
    // Motor Out/Input
    // std::cout << "Left MO: " << m_leftOutput.Get() << std::endl;
    // std::cout << "Left MI: " << m_leftMotorInput.Get() << std::endl;
    // std::cout << "Right MO: " << m_rightOutput.Get() << std::endl;
    // std::cout << "Right MI: " << m_rightMotorInput.Get() << std::endl;

    // Gyro Values
    // std::cout << "Gyro Rate: " << m_gyro.GetRate();
    // std::cout << " Angle: " << m_gyro.GetAngle() << std::endl;

    // Velocity and Position
    // std::cout << "Left Velocity: " << m_leftGrbx.GetSpeed() << std::endl;
    // std::cout << "Right Velocity: " << m_rightGrbx.GetSpeed() << std::endl;
    // std::cout << "Left Position: " << m_leftGrbx.GetPosition()
    // << "Right: " << m_rightGrbx.GetPosition() << std::endl;

    // PID
    // std::cout << "Pos PID: " << m_posPID.Get() << std::endl;
    // std::cout << "Velocity PID: " << m_velPID.Get() << std::endl;
    // std::cout << "Angle PID: " << m_anglePID.Get() << std::endl;
    // std::cout << "Angle Error: " << m_angleError.Get() << std::endl;
    // std::cout << "Pos Error: " << m_posError.Get() << std::endl;
    // std::cout << "Rotate: " << m_rotatePID.Get() << std::endl;

    // References
    // std::cout << "References: Position: " << m_posRef.Get()
    // << "Angle: " << m_angleRef.Get() << std::endl;
}
