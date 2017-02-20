// Copyright (c) FRC Team 3512, Spartatroniks 2016-2017. All Rights Reserved.

#pragma once

#include <memory>

#include <ADXRS450_Gyro.h>
#include <Encoder.h>

#include "../Constants.hpp"
#include "../CtrlSys/FuncNode.hpp"
#include "../CtrlSys/LinearDigitalFilter.h"
#include "../CtrlSys/Output.hpp"
#include "../CtrlSys/PIDNode.hpp"
#include "../CtrlSys/RefInput.hpp"
#include "../CtrlSys/Sensor.hpp"
#include "../CtrlSys/SumNode.hpp"
#include "../SM/StateMachine.hpp"
#include "../Utility.hpp"
#include "GearBox.hpp"
#include "SubsystemBase.hpp"

class GearBox;

/**
 * Provides an interface for this year's drive train
 */
class DriveTrain : public SubsystemBase {
public:
    DriveTrain();
    virtual ~DriveTrain() = default;

    int32_t GetLeftRaw() const;
    int32_t GetRightRaw() const;

    /* Drives robot with given speed and turn values [-1..1].
     * This is a convenience function for use in Operator Control.
     */
    void Drive(double throttle, double turn, bool isQuickTurn = false);

    // Sets joystick deadband
    void SetDeadband(double band);

    // Set encoder distances to 0
    void ResetEncoders();

    // Directly set wheel speeds [0..1] (see GearBox::SetManual(double))
    void SetLeftManual(double value);
    void SetRightManual(double value);

    // Returns encoder distances
    double GetLeftDisplacement() const;
    double GetRightDisplacement() const;

    // Returns encoder rates
    double GetLeftRate() const;
    double GetRightRate() const;

    void SetVelocityReference(double velocity);

    double GetVelocity();

    void StartClosedLoop();
    void StopClosedLoop();

    // Returns encoder PID loop setpoints
    double GetVelSetpoint() const;
    double GetRotateSetpoint() const;

    // Return gyro's angle
    double GetAngle() const;

    double GetRate() const;

    double GetFilteredRate();

    void SetRotationReference(double reference);

    // Resets gyro
    void ResetGyro();

    void CalibrateGyro();

    void Debug();

private:
    double m_deadband = k_joystickDeadband;
    double m_sensitivity;

    // Cheesy Drive variables
    double m_oldTurn = 0.0;
    double m_quickStopAccumulator = 0.0;
    double m_negInertiaAccumulator = 0.0;

    // Control system references
    RefInput m_rotateRef{0.0};
    RefInput m_velRef{0.0};

    // Rotation rate PID
    ADXRS450_Gyro m_gyro;
    FuncNode m_rotateRate{[&] { return m_gyro.GetRate(); }};
    LinearDigitalFilter m_rotateFilter =
        LinearDigitalFilter::SinglePoleIIR(&m_rotateRate, 0.35, 0.005);
    SumNode m_rotateError{&m_rotateRef, true, &m_rotateFilter, false};
    PIDNode m_rotatePID{k_rotateP, k_rotateI, k_rotateD, &m_rotateError};

    // Gearboxes used in velocity PID
    GearBox m_leftGrbx{-1, -1, -1, k_leftDriveMasterID, k_leftDriveSlaveID};
    GearBox m_rightGrbx{-1, -1, -1, k_rightDriveMasterID, k_rightDriveSlaveID};

    // Velocity PID
    FuncNode m_velCalc{
        [&] { return (m_leftGrbx.GetSpeed() + m_rightGrbx.GetSpeed()) / 2.0; }};
    SumNode m_velError{&m_velRef, true, &m_velCalc, false};
    PIDNode m_velPID{k_speedP, k_speedI, k_speedD, &m_velError};

    // Combine outputs for left motor
    GainNode m_leftFeedForward{1 / k_rotateMaxSpeed, &m_rotateRef};
    SumNode m_leftMotorInput{&m_velPID,          true, &m_rotatePID, true,
                             &m_leftFeedForward, true};
    Output m_leftOutput{&m_leftMotorInput, &m_leftGrbx, 0.005};

    // Combine outputs for right motor
    GainNode m_rightFeedForward{1 / k_rotateMaxSpeed, &m_rotateRef};
    SumNode m_rightMotorInput{&m_velPID,           true, &m_rotatePID, false,
                              &m_rightFeedForward, false};
    Output m_rightOutput{&m_rightMotorInput, &m_rightGrbx, 0.005};
};
