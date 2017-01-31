// Copyright (c) FRC Team 3512, Spartatroniks 2017. All Rights Reserved.

#pragma once

#include <memory>

#include <HAL/HAL.h>
#include <PIDInterface.h>
#include <PIDOutput.h>

#include "NodeBase.hpp"
#include "Output.hpp"
#include "PIDNode.hpp"
#include "RefInput.hpp"
#include "SumNode.hpp"

class PIDController : public frc::PIDInterface {
public:
    PIDController(double Kp, double Ki, double Kd, NodeBase* input,
                  PIDOutput* output, double period = 0.05);
    PIDController(double Kp, double Ki, double Kd, NodeBase* feedforward,
                  NodeBase* input, PIDOutput* output, double period = 0.05);
    virtual ~PIDController() = default;

    void SetPID(double p, double i, double d) override;
    double GetP() const override;
    double GetI() const override;
    double GetD() const override;

    void SetSetpoint(double setpoint) override;
    double GetSetpoint() const override;

    void Enable() override;
    void Disable() override;
    bool IsEnabled() const override;

    void Reset() override;

private:
    RefInput m_refInput{0.0};
    std::unique_ptr<SumNode> m_sum;
    std::unique_ptr<PIDNode> m_pid;
    std::unique_ptr<Output> m_output;
    bool m_enabled = false;
};
