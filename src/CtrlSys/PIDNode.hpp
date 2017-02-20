// Copyright (c) FRC Team 3512, Spartatroniks 2017. All Rights Reserved.

#pragma once

#include <memory>

#include "DerivativeNode.hpp"
#include "GainNode.hpp"
#include "IntegralNode.hpp"
#include "NodeBase.hpp"
#include "SumNode.hpp"

/**
 * A PID controller implementation based on the control system diagram
 * framework. This is more general than the PIDController as its output can be
 * chained with other nodes that are not actuators.
 *
 * For a simple closed-loop PID controller, see PIDController.
 */
class PIDNode : public NodeBase {
public:
    PIDNode(double Kp, double Ki, double Kd, NodeBase* input,
            double period = 0.05);
    PIDNode(double Kp, double Ki, double Kd, NodeBase* feedforward,
            NodeBase* input, double period = 0.05);
    virtual ~PIDNode() = default;

    double Get() override;

    void SetPID(double p, double i, double d);
    double GetP() const;
    double GetI() const;
    double GetD() const;

    void Reset();

    double Total();
    void SetOutputRange(double minU, double maxU);

private:
    std::unique_ptr<GainNode> m_P;
    std::unique_ptr<IntegralNode> m_I;
    std::unique_ptr<DerivativeNode> m_D;
    std::unique_ptr<SumNode> m_sum;

    double m_minU = -1.0;
    double m_maxU = 1.0;
};
