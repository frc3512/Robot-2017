// Copyright (c) FRC Team 3512, Spartatroniks 2017. All Rights Reserved.

#pragma once

#include "NodeBase.hpp"

/**
 * Represents an integrator in a control system diagram.
 */
class IntegralNode : public NodeBase {
public:
    IntegralNode(double K, NodeBase* input, double period = 0.05);
    virtual ~IntegralNode() = default;

    double Get() override;

    void SetGain(double K);
    double GetGain() const;

    void Reset();

private:
    double m_K;
    NodeBase* m_input;
    double m_period;

    double m_total = 0.0;
};
