// Copyright (c) 2017-2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <units/units.h>

#include <frc/PIDOutput.h>
#include <frc/ctrlsys/FuncNode.h>
#include <frc/ctrlsys/INode.h>
#include <frc/ctrlsys/Output.h>
#include <frc/ctrlsys/OutputGroup.h>
#include <frc/ctrlsys/PIDNode.h>
#include <frc/ctrlsys/SumNode.h>

namespace frc {

/**
 * A feedback controller for a differential-drive robot. A differential-drive
 * robot has left and right wheels separated by an arbitrary width.
 *
 * A forward distance controller and angle controller are run in parallel and
 * their outputs are composed to drive each wheel. Since the forward controller
 * uses the average distance of the two sides while the angle controller uses
 * the difference between them, the controllers act independently on the drive
 * base and can thus be tuned separately.
 *
 * If you don't have a gyroscope for an angle sensor, the following equation can
 * be used in a FuncNode to estimate it.
 *
 * angle = (right - left) / width * 180 / pi
 *
 * where "right" is the right encoder reading, "left" is the left encoder
 * reading, "width" is the width of the robot in the same units as the
 * encoders, and "angle" is the angle of the robot in degrees. We recommend
 * passing this angle estimation through a low-pass filter (see LinearFilter).
 *
 * Set the position and angle PID constants via GetPositionPID()->SetPID() and
 * GetAnglePID()->SetPID() before enabling this controller.
 */
class DiffDriveController {
public:
    DiffDriveController(INode& positionRef, INode& angleRef, INode& leftEncoder,
                        INode& rightEncoder, INode& angleSensor, bool clockwise,
                        PIDOutput& leftMotor, PIDOutput& rightMotor,
                        units::second_t period = INode::kDefaultPeriod);

    void Enable();
    void Disable();

    PIDNode& GetPositionPID();
    PIDNode& GetAnglePID();

    double GetPosition();
    double GetAngle();

    void SetPositionTolerance(double tolerance, double deltaTolerance);
    void SetAngleTolerance(double tolerance, double deltaTolerance);

    bool AtPosition() const;
    bool AtAngle() const;

private:
    // Control system references
    INode& m_positionRef;
    INode& m_angleRef;

    // Encoders
    INode& m_leftEncoder;
    INode& m_rightEncoder;

    // Angle sensor (e.g., gyroscope)
    INode& m_angleSensor;
    bool m_clockwise;

    // Motors
    PIDOutput& m_leftMotor;
    PIDOutput& m_rightMotor;

    // Position PID
    FuncNode m_positionCalc{[&] {
        return (m_leftEncoder.GetOutput() + m_rightEncoder.GetOutput()) / 2.0;
    }};
    SumNode m_positionError{m_positionRef, true, m_positionCalc, false};
    PIDNode m_positionPID{0.0, 0.0, 0.0, m_positionError};

    // Angle PID
    SumNode m_angleError{m_angleRef, true, m_angleSensor, false};
    PIDNode m_anglePID{0.0, 0.0, 0.0, m_angleError};

    // Combine outputs for left motor
    SumNode m_leftMotorInput;
    Output m_leftOutput;

    // Combine outputs for right motor
    SumNode m_rightMotorInput;
    Output m_rightOutput;

    OutputGroup m_outputs;
    units::second_t m_period;
};

}  // namespace frc
