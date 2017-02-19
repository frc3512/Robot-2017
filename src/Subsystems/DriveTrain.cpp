// Copyright (c) FRC Team 3512, Spartatroniks 2016-2017. All Rights Reserved.

#include "DriveTrain.hpp"

#include <cmath>
#include <iostream>

DriveTrain::DriveTrain() {
    m_sensitivity = k_lowGearSensitive;

    m_rightGrbx.SetInverted(true);

    m_leftGrbx.SetSensorDirection(true);

    m_leftGrbx.GetMaster()->SetFeedbackDevice(CANTalon::QuadEncoder);
    m_rightGrbx.GetMaster()->SetFeedbackDevice(CANTalon::QuadEncoder);

    m_leftGrbx.SetDistancePerPulse(k_driveDpP);
    m_rightGrbx.SetDistancePerPulse(k_driveDpP);

    m_leftGrbx.Set(0.0);
    m_rightGrbx.Set(0.0);
}

int32_t DriveTrain::GetLeftRaw() const { return m_leftGrbx.Get(); }

int32_t DriveTrain::GetRightRaw() const { return m_rightGrbx.Get(); }

void DriveTrain::Drive(double throttle, double turn, bool isQuickTurn) {
    // Modified Cheesy Drive; base code courtesy of FRC Team 254

    throttle *= -1;

    // Limit values to [-1 .. 1]
    throttle = Limit(throttle, 1.0);
    turn = Limit(turn, 1.0);

    /* Apply joystick deadband
     * (Negate turn since joystick X-axis is reversed)
     */
    throttle = ApplyDeadband(throttle, m_deadband);
    turn = ApplyDeadband(turn, m_deadband);

    double turnNonLinearity = k_turnNonLinearity;

    /* Apply a sine function that's scaled to make turning sensitivity feel
     * better. turnNonLinearity should never be zero, but can be close.
     */
    turn = std::sin(M_PI / 2.0 * turnNonLinearity * turn) /
           std::sin(M_PI / 2.0 * turnNonLinearity);

    double angularPower = 0.0;
    double linearPower = throttle;
    double leftPwm = linearPower, rightPwm = linearPower;

    double negInertia = turn - m_oldTurn;
    m_oldTurn = turn;

    // Negative inertia!
    double negInertiaScalar;
    if (turn * negInertia > 0) {
        negInertiaScalar = k_inertiaDampen;
    } else {
        if (std::fabs(turn) > 0.65) {
            negInertiaScalar = k_inertiaHighTurn;
        } else {
            negInertiaScalar = k_inertiaLowTurn;
        }
    }

    m_negInertiaAccumulator +=
        negInertia * negInertiaScalar;  // adds negInertiaPower

    // Apply negative inertia
    turn += m_negInertiaAccumulator;
    if (m_negInertiaAccumulator > 1) {
        m_negInertiaAccumulator -= 1;
    } else if (m_negInertiaAccumulator < -1) {
        m_negInertiaAccumulator += 1;
    } else {
        m_negInertiaAccumulator = 0;
    }

    // QuickTurn!
    if (isQuickTurn) {
        if (std::fabs(linearPower) < 0.2) {
            double alpha = 0.1;
            m_quickStopAccumulator = (1 - alpha) * m_quickStopAccumulator +
                                     alpha * Limit(turn, 1.0) * 5;
        }

        angularPower = turn;
    } else {
        angularPower =
            std::fabs(throttle) * turn * m_sensitivity - m_quickStopAccumulator;
        if (m_quickStopAccumulator > 1) {
            m_quickStopAccumulator -= 1;
        } else if (m_quickStopAccumulator < -1) {
            m_quickStopAccumulator += 1;
        } else {
            m_quickStopAccumulator = 0.0;
        }
    }

    // Adjust straight path for turn
    leftPwm += angularPower;
    rightPwm -= angularPower;

    // Limit PWM bounds to [-1..1]
    if (leftPwm > 1.0) {
        // If overpowered turning enabled
        if (isQuickTurn) {
            rightPwm -= (leftPwm - 1.0);
        }

        leftPwm = 1.0;
    } else if (rightPwm > 1.0) {
        // If overpowered turning enabled
        if (isQuickTurn) {
            leftPwm -= (rightPwm - 1.0);
        }

        rightPwm = 1.0;
    } else if (leftPwm < -1.0) {
        // If overpowered turning enabled
        if (isQuickTurn) {
            rightPwm += (-leftPwm - 1.0);
        }

        leftPwm = -1.0;
    } else if (rightPwm < -1.0) {
        // If overpowered turning enabled
        if (isQuickTurn) {
            leftPwm += (-rightPwm - 1.0);
        }

        rightPwm = -1.0;
    }
    m_leftGrbx.Set(leftPwm);
    m_rightGrbx.Set(rightPwm);
}

void DriveTrain::SetDeadband(double band) { m_deadband = band; }

void DriveTrain::ResetEncoders() {
    m_leftGrbx.ResetEncoder();
    m_rightGrbx.ResetEncoder();
}
void DriveTrain::SetLeftManual(double value) { m_leftGrbx.Set(value); }

void DriveTrain::SetRightManual(double value) { m_rightGrbx.Set(value); }

double DriveTrain::GetLeftDisplacement() const {
    return m_leftGrbx.GetPosition();
}

double DriveTrain::GetRightDisplacement() const {
    return m_rightGrbx.GetPosition();
}

double DriveTrain::GetPosition() {
	return (m_rightGrbx.GetPosition() + m_leftGrbx.GetPosition())/2;
}

double DriveTrain::GetLeftRate() const { return m_leftGrbx.GetSpeed(); }

double DriveTrain::GetRightRate() const { return m_rightGrbx.GetSpeed(); }

void DriveTrain::SetPositionReference(double position) {
    m_positionRef.Set(position);
}

double DriveTrain::GetVelocity() {
    return (m_leftGrbx.GetSpeed() + m_rightGrbx.GetSpeed()) / 2;
}

void DriveTrain::StartClosedLoop() {
    m_leftOutput.Start();
    m_rightOutput.Start();
}

void DriveTrain::StopClosedLoop() {
    m_leftOutput.Stop();
    m_rightOutput.Stop();
}

double DriveTrain::GetPositionSetpoint() const { return m_positionRef.Get(); }

double DriveTrain::GetAngleSetpoint() const { return m_angleRef.Get(); }

double DriveTrain::GetAngle() const { return m_gyro.GetAngle(); }

double DriveTrain::GetRate() const { return m_gyro.GetRate(); }

double DriveTrain::GetFilteredRate() { return m_rotateFilter.Get(); }

void DriveTrain::SetAngleReference(double reference) {
    m_angleRef.Set(reference);
}

double DriveTrain::GetRotationReference() { return m_angleGain.Get(); }

void DriveTrain::ResetGyro() { m_gyro.Reset(); }

void DriveTrain::CalibrateGyro() { m_gyro.Calibrate(); }

void DriveTrain::Debug() {
    // std::cout << "Left MO: " << m_leftOutput.Get() << "  Left MI: " <<
    // m_leftMotorInput.Get() << std::endl;
    // std::cout << "Right MO: " << m_rightOutput.Get() << "  Right MI: " <<
    // m_rightMotorInput.Get() << std::endl;
    // std::cout << "Rotate: " << m_rotatePID.Get() << std::endl;
    std::cout << "Gyro: " << m_gyro.GetRate() << std::endl;
    std::cout << "Left Rate: " << m_leftEncoder.Get() << std::endl;
    std::cout << "Right Rate: " << m_rightEncoder.Get() << std::endl;
    std::cout << "Left Pos: " << m_leftGrbx.GetPosition() << std::endl;
    std::cout << "Right Pos: " << m_rightGrbx.GetPosition() << std::endl;
    // std::cout << "Velocity PID: " << m_velPID.Get() << std::endl;
}
