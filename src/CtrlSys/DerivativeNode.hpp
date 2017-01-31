// Copyright (c) FRC Team 3512, Spartatroniks 2017. All Rights Reserved.

#pragma once

#include "NodeBase.hpp"

/**
 * Returns the integral of the input node's output.
 */
class DerivativeNode : public NodeBase {
public:
    DerivativeNode(double K, NodeBase* input, double period = 0.05);
    virtual ~DerivativeNode() = default;

    double Get() override;

    void SetGain(double K);
    double GetGain() const;

    void Reset();

private:
    double m_K;
    NodeBase* m_input;
    double m_period;

    double m_prevInput = 0.0;
};
