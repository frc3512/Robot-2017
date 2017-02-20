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

    void SetPositionReference(double position);

    void SetAngleReference(double angle);

    double GetPosition();

    void StartClosedLoop();
    void StopClosedLoop();

    // Returns encoder PID loop setpoints
    double GetPosReference() const;
    double GetAngleReference() const;

    // Return gyro's angle
    double GetAngle() const;

    double GetRate() const;

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

    ADXRS450_Gyro m_gyro;
    FuncNode m_angleSensor{[&](auto& inputs) { return m_gyro.GetAngle(); }};
    // LinearDigitalFilter m_rotateFilter =
    //     LinearDigitalFilter::SinglePoleIIR(&m_rotateRate, 0.35, 0.005);
    RefInput m_angleRef{0.0};
    SumNode m_angleError{&m_angleRef, true, &m_angleSensor, false};
    PIDNode m_anglePID{k_angleP, k_angleI, k_angleD, &m_angleError};

    GearBox m_leftGrbx{-1, -1, -1, k_leftDriveMasterID, k_leftDriveSlaveID};
    GearBox m_rightGrbx{-1, -1, -1, k_rightDriveMasterID, k_rightDriveSlaveID};

    FuncNode m_leftEncoder{
        [&](auto& inputs) { return m_leftGrbx.GetPosition(); }};

    FuncNode m_rightEncoder{
        [&](auto& inputs) { return m_rightGrbx.GetPosition(); }};

    RefInput m_posRef{0.0};
    FuncNode m_posCalc{[](auto& inputs) {
                           return (inputs[0]->Get() + inputs[1]->Get()) / 2.0;
                       },
                       &m_leftEncoder, &m_rightEncoder};
    SumNode m_posError{&m_posRef, true, &m_posCalc, false};
    PIDNode m_posPID{k_posP, k_posI, k_posD, &m_posError};

    SumNode m_leftMotorInput{&m_posPID, true, &m_anglePID, true};
    Output m_leftOutput{&m_leftMotorInput, &m_leftGrbx, 0.005};

    SumNode m_rightMotorInput{&m_posPID, true, &m_anglePID, false};
    Output m_rightOutput{&m_rightMotorInput, &m_rightGrbx, 0.005};
};
