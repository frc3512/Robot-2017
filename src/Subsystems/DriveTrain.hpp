// Copyright (c) FRC Team 3512, Spartatroniks 2016-2017. All Rights Reserved.

#pragma once

#include <memory>

#include <ADXRS450_Gyro.h>
#include <Encoder.h>

#include "../Constants.hpp"
#include "../CtrlSys/FuncNode.hpp"
#include "../CtrlSys/GainNode.hpp"
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

    // Takes average of right and left displacements
    double GetPosition();

    void SetPositionReference(double position);

    // Returns encoder rates
    double GetLeftRate() const;
    double GetRightRate() const;

    // Takes average of right and left rates
    double GetVelocity();

    // Enables feedback
    void StartClosedLoop();
    void StopClosedLoop();

    // Returns encoder PID loop setpoints
    double GetPositionSetpoint() const;
    double GetAngleSetpoint() const;

    // Return gyro's angle
    double GetAngle() const;

    double GetRate() const;

    double GetFilteredRate();

    void SetAngleReference(double reference);

    double GetRotationReference();

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
    RefInput m_angleRef{0.0};
    RefInput m_positionRef{0.0};

    // Angle PID
    ADXRS450_Gyro m_gyro;
    FuncNode m_gyroAngle{[&] { return m_gyro.GetAngle(); }};
    SumNode m_angleError{&m_angleRef, true, &m_gyroAngle, false};
    PIDNode m_anglePID{k_angleP, k_angleI, k_angleD, &m_angleError};
    GainNode m_angleGain{k_rotateMaxSpeed, &m_anglePID};

    // Rotation rate PID
    FuncNode m_rotateRate{[&] { return m_gyro.GetRate(); }};
    LinearDigitalFilter m_rotateFilter =
        LinearDigitalFilter::SinglePoleIIR(&m_rotateRate, 0.35, 0.005);
    SumNode m_rotateError{&m_angleGain, true, &m_rotateFilter, false};
    PIDNode m_rotatePID{k_rotateP, k_rotateI, k_rotateD, &m_rotateError};

    // Gearboxes used in position/velocity PID
    GearBox m_leftGrbx{-1, -1, -1, k_leftDriveMasterID, k_leftDriveSlaveID};
    GearBox m_rightGrbx{-1, -1, -1, k_rightDriveMasterID, k_rightDriveSlaveID};

    // Position PID
    FuncNode m_averagePos{[&] {
        return (m_leftGrbx.GetPosition() + m_rightGrbx.GetPosition()) / 2;
    }};
    SumNode m_positionError{&m_positionRef, true, &m_averagePos, false};
    PIDNode m_positionPID{k_angleP, k_angleI, k_angleD, &m_positionError};
    GainNode m_positionGain{k_driveMaxSpeed, &m_positionPID};

    // Velocity PID
    FuncNode m_velCalc{
        [&] { return (m_leftGrbx.GetSpeed() + m_rightGrbx.GetSpeed()) / 2.0; }};
    SumNode m_velError{&m_positionGain, true, &m_velCalc, false};
    PIDNode m_velPID{k_speedP, k_speedI, k_speedD, &m_velError};

    // Combine outputs for left motor
    GainNode m_leftFeedForward{1 / k_rotateMaxSpeed, &m_angleGain};
    SumNode m_leftMotorInput{&m_velPID,          true, &m_rotatePID, true,
                             &m_leftFeedForward, true};
    Output m_leftOutput{&m_leftMotorInput, &m_leftGrbx, 0.005};

    // Combine outputs for right motor
    GainNode m_rightFeedForward{1 / k_rotateMaxSpeed, &m_angleGain};
    SumNode m_rightMotorInput{&m_velPID,           true, &m_rotatePID, false,
                              &m_rightFeedForward, false};
    Output m_rightOutput{&m_rightMotorInput, &m_rightGrbx, 0.005};
};
