// Copyright (c) FRC Team 3512, Spartatroniks 2017. All Rights Reserved.

#include "PIDNode.hpp"

/**
 * Allocate a PID object with the given constants for P, I, D.
 *
 * @param Kp the proportional coefficient
 * @param Ki the integral coefficient
 * @param Kd the derivative coefficient
 * @param input the node that is used to get values
 * @param period the loop time for doing calculations. This particularly
 *               effects calculations of the integral and differental terms.
 *               The default is 50ms.
 */
PIDNode::PIDNode(double Kp, double Ki, double Kd, NodeBase* input,
                 double period) {
    m_P = std::make_unique<GainNode>(Kp, input);
    m_I = std::make_unique<IntegralNode>(Ki, input, period);
    m_D = std::make_unique<DerivativeNode>(Kd, input, period);
    m_sum = std::make_unique<SumNode>(m_P.get(), true, m_I.get(), true,
                                      m_D.get(), true);
}

/**
 * Allocate a PID object with the given constants for P, I, D.
 *
 * @param Kp the proportional coefficient
 * @param Ki the integral coefficient
 * @param Kd the derivative coefficient
 * @param feedforward node to use for feedforward calculations
 * @param input the node that is used to get values
 * @param period the loop time for doing calculations. This particularly
 *               effects calculations of the integral and differental terms.
 *               The default is 50ms.
 */
PIDNode::PIDNode(double Kp, double Ki, double Kd, NodeBase* feedforward,
                 NodeBase* input, double period) {
    m_P = std::make_unique<GainNode>(Kp, input);
    m_I = std::make_unique<IntegralNode>(Ki, input, period);
    m_D = std::make_unique<DerivativeNode>(Kd, input, period);
    m_sum = std::make_unique<SumNode>(m_P.get(), true, m_I.get(), true,
                                      m_D.get(), true, feedforward, true);
}

double PIDNode::Get() {
    double sum = m_sum->Get();

    if (sum > 1.0) {
        return 1.0;
    } else if (sum < -1.0) {
        return -1.0;
    } else {
        return sum;
    }
}

void PIDNode::SetPID(double p, double i, double d) {
    m_P->SetGain(p);
    m_I->SetGain(i);
    m_D->SetGain(d);
}

double PIDNode::GetP() const { return m_P->GetGain(); }

double PIDNode::GetI() const { return m_I->GetGain(); }

double PIDNode::GetD() const { return m_D->GetGain(); }

void PIDNode::Reset() {
    m_I->Reset();
    m_D->Reset();
}

double PIDNode::Total() { return m_I->Get(); }
