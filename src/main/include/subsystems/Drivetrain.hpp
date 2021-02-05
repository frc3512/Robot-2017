// Copyright (c) 2016-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <frc/ADXRS450_Gyro.h>
#include <frc/Encoder.h>
#include <frc/ctrlsys/FuncNode.h>
#include <frc/ctrlsys/RefInput.h>
#include <frc/drive/DifferentialDrive.h>

#include "CANEncoder.hpp"
#include "Constants.hpp"
#include "DiffDriveController.hpp"
#include "TalonSRXGroup.hpp"

/**
 * Provides an interface for this year's drive train
 */
class Drivetrain {
public:
    using WPI_TalonSRX = ctre::phoenix::motorcontrol::can::WPI_TalonSRX;

    Drivetrain();

    int32_t GetLeftRaw() const;
    int32_t GetRightRaw() const;

    /* Drives robot with given speed and turn values [-1..1].
     * This is a convenience function for use in Operator Control.
     */
    void Drive(double throttle, double turn, bool isQuickTurn = false);

    // Set encoder distances to 0
    void ResetEncoders();

    // Directly set wheel speeds [0..1] (see GearBox::SetManual(double))
    void SetLeftManual(double value);
    void SetRightManual(double value);

    // Returns encoder distances
    double GetLeftDisplacement();
    double GetRightDisplacement();

    // Returns encoder rates
    double GetLeftRate();
    double GetRightRate();

    // Returns robot's current position
    double GetPosition();

    // Return gyro's angle
    double GetAngle();

    // Return gyro's rate
    double GetAngularRate() const;

    // Starts and stops PID loops
    void StartClosedLoop();
    void StopClosedLoop();

    // Sets encoder PID setpoints
    void SetPositionReference(double position);
    void SetAngleReference(double angle);

    // Returns encoder PID loop references
    double GetPosReference() const;
    double GetAngleReference() const;

    // Returns whether or not robot has reached reference
    bool PosAtReference() const;
    bool AngleAtReference() const;

    // Resets gyro
    void ResetGyro();

    // Calibrates gyro
    void CalibrateGyro();

    // Sends print statements for debugging purposes
    void Debug();

private:
    // Left gearbox used in position PID
    WPI_TalonSRX m_leftFront{kLeftDriveMasterID};
    WPI_TalonSRX m_leftRear{kLeftDriveSlaveID};
    TalonSRXGroup m_leftGrbx{m_leftFront, m_leftRear};
    CANEncoder m_leftEncoder{m_leftFront, kDriveDpP};

    // Right gearbox used in position PID
    WPI_TalonSRX m_rightFront{kRightDriveMasterID};
    WPI_TalonSRX m_rightRear{kRightDriveSlaveID};
    TalonSRXGroup m_rightGrbx{m_rightFront, m_rightRear};
    CANEncoder m_rightEncoder{m_rightFront, kDriveDpP};

    frc::DifferentialDrive m_drive{m_leftGrbx, m_rightGrbx};

    // Gyro used for angle PID
    frc::ADXRS450_Gyro m_gyro;

    // Control system references
    frc::RefInput m_posRef{0.0};
    frc::RefInput m_angleRef{0.0};

    // Sensor adapters
    frc::FuncNode m_leftEncoderDistance{
        [this] { return m_leftEncoder.GetDistance(); }};
    frc::FuncNode m_rightEncoderDistance{
        [this] { return m_rightEncoder.GetDistance(); }};
    frc::FuncNode m_angleSensor{[this] { return m_gyro.GetAngle(); }};

    frc::DiffDriveController m_controller{m_posRef,
                                          m_angleRef,
                                          m_leftEncoderDistance,
                                          m_rightEncoderDistance,
                                          m_angleSensor,
                                          true,
                                          m_leftGrbx,
                                          m_rightGrbx};
};
