// Copyright (c) FRC Team 3512, Spartatroniks 2017. All Rights Reserved.

#include "PIDController.hpp"

/**
 * Allocate a PID object with the given constants for P, I, D.
 *
 * @param Kp the proportional coefficient
 * @param Ki the integral coefficient
 * @param Kd the derivative coefficient
 * @param input the node that is used to get values
 * @param output The PIDOutput object that is set to the output value
 * @param period the loop time for doing calculations. This particularly
 *               effects calculations of the integral and differental terms.
 *               The default is 50ms.
 */
PIDController::PIDController(double Kp, double Ki, double Kd, NodeBase* input,
                             PIDOutput* output, double period) {
    m_sum = std::make_unique<SumNode>(&m_refInput, true, input, false);
    m_pid = std::make_unique<PIDNode>(Kp, Ki, Kd, m_sum.get(), period);
    m_output = std::make_unique<Output>(m_pid.get(), output, period);

    static int instances = 0;
    instances++;
    HAL_Report(HALUsageReporting::kResourceType_PIDController, instances);
}

/**
 * Allocate a PID object with the given constants for P, I, D.
 *
 * @param Kp the proportional coefficient
 * @param Ki the integral coefficient
 * @param Kd the derivative coefficient
 * @param feedforward node to use for feedforward calculations
 * @param input the node that is used to get values
 * @param output The PIDOutput object that is set to the output value
 * @param period the loop time for doing calculations. This particularly
 *               effects calculations of the integral and differental terms.
 *               The default is 50ms.
 */
PIDController::PIDController(double Kp, double Ki, double Kd,
                             NodeBase* feedforward, NodeBase* input,
                             PIDOutput* output, double period) {
    m_sum = std::make_unique<SumNode>(&m_refInput, true, input, false);
    m_pid =
        std::make_unique<PIDNode>(Kp, Ki, Kd, feedforward, m_sum.get(), period);
    m_output = std::make_unique<Output>(m_pid.get(), output, period);

    static int instances = 0;
    instances++;
    HAL_Report(HALUsageReporting::kResourceType_PIDController, instances);
}

void PIDController::SetPID(double p, double i, double d) {
    m_pid->SetPID(p, i, d);
}

double PIDController::GetP() const { return m_pid->GetP(); }

double PIDController::GetI() const { return m_pid->GetI(); }

double PIDController::GetD() const { return m_pid->GetD(); }

void PIDController::SetSetpoint(double setpoint) { m_refInput.Set(setpoint); }

double PIDController::GetSetpoint() const { return m_refInput.Get(); }

void PIDController::Enable() {
    m_enabled = true;
    m_output->Start();
}

void PIDController::Disable() {
    m_output->Stop();
    m_enabled = false;
}

bool PIDController::IsEnabled() const { return m_enabled; }

void PIDController::Reset() {
    Disable();

    m_pid->Reset();
}
