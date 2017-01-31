// Copyright (c) FRC Team 3512, Spartatroniks 2016-2017. All Rights Reserved.

#pragma once

#include <memory>

#include <ADXRS450_Gyro.h>

#include "../Constants.hpp"
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

    void EnablePID();
    void DisablePID();

    // Returns encoder PID loop setpoints
    double GetLeftSetpoint() const;
    double GetRightSetpoint() const;

private:
    double m_deadband = k_joystickDeadband;
    double m_sensitivity;

    // Cheesy Drive variables
    double m_oldTurn = 0.0;
    double m_quickStopAccumulator = 0.0;
    double m_negInertiaAccumulator = 0.0;

    ADXRS450_Gyro m_gyro;
    Sensor m_rotateSensor{&m_gyro};
    RefInput m_rotateRef{0.0};
    SumNode m_rotatePIDInput{&m_rotateRef, true, &m_rotateSensor, false};
    PIDNode m_rotatePID{k_rotateP, k_rotateI, k_rotateD, &m_rotatePIDInput};

    GearBox m_leftGrbx{-1, -1, -1, k_leftDriveMasterID, k_leftDriveSlaveID};
    Sensor m_leftSensor{&m_leftGrbx};
    RefInput m_leftRef{0.0};
    SumNode m_leftPIDInput{&m_leftRef, true, &m_leftSensor, false};
    PIDNode m_leftPID{k_speedP, k_speedI, k_speedD, &m_leftPIDInput};

    GearBox m_rightGrbx{-1, -1, -1, k_rightDriveMasterID, k_rightDriveSlaveID};
    Sensor m_rightSensor{&m_rightGrbx};
    RefInput m_rightRef{0.0};
    SumNode m_rightPIDInput{&m_rightRef, true, &m_rightSensor, false};
    PIDNode m_rightPID{k_speedP, k_speedI, k_speedD, &m_rightPIDInput};

    SumNode m_leftSum{&m_leftPID, true, &m_rotatePID, false};
    Output m_leftOutput{&m_leftSum, &m_leftGrbx};

    SumNode m_rightSum{&m_rightPID, true, &m_rotatePID, true};
    Output m_rightOutput{&m_leftSum, &m_leftGrbx};
};
