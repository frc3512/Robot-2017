// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <limits>

#include <units/units.h>

#include "frc/PIDOutput.h"
#include "frc/ctrlsys/GainNode.h"
#include "frc/ctrlsys/INode.h"
#include "frc/ctrlsys/Output.h"
#include "frc/ctrlsys/PIDNode.h"
#include "frc/ctrlsys/RefInput.h"
#include "frc/ctrlsys/SumNode.h"

namespace frc3512 {

/**
 * Class implements a PID Control Loop.
 *
 * Creates a separate thread which reads the given PIDSource and takes
 * care of the integral calculations, as well as writing the given PIDOutput.
 *
 * This feedback controller runs in discrete time, so time deltas are not used
 * in the integral and derivative calculations. Therefore, the sample rate
 * affects the controller's behavior for a given set of PID constants.
 */
class PIDController {
public:
    PIDController(double Kp, double Ki, double Kd, frc::INode& input,
                  frc::PIDOutput& output,
                  units::second_t period = frc::INode::kDefaultPeriod);
    PIDController(double Kp, double Ki, double Kd, double Kff,
                  frc::INode& input, frc::PIDOutput& output,
                  units::second_t period = frc::INode::kDefaultPeriod);

    PIDController(const PIDController&) = delete;
    PIDController& operator=(const PIDController) = delete;

    void SetPID(double Kp, double Ki, double Kd);
    void SetPID(double Kp, double Ki, double Kd, double Kff);
    double GetP() const;
    double GetI() const;
    double GetD() const;
    double GetF() const;

    void SetContinuous(bool continuous = true);
    void SetInputRange(double minimumInput, double maximumInput);
    void SetOutputRange(double minimumOutput, double maximumOutput);
    void SetIZone(double maxErrorMagnitude);

    void SetSetpoint(double setpoint);
    double GetSetpoint() const;

    void SetAbsoluteTolerance(
        double tolerance,
        double deltaTolerance = std::numeric_limits<double>::infinity());
    double GetError(void);
    bool OnTarget() const;

    void Enable(void);
    void Disable(void);
    bool IsEnabled() const;
    void SetEnabled(bool enable);

    void Reset();

private:
    frc::RefInput m_refInput{0.0};
    frc::GainNode m_feedforward{0.0, m_refInput};
    frc::SumNode m_sum;
    frc::PIDNode m_pid;
    frc::Output m_output;
    double m_tolerance = 0.05;
    bool m_enabled = false;
};

}  // namespace frc
