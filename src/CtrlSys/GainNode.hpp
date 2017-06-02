// Copyright (c) 2017 FRC Team 3512. All Rights Reserved.

#pragma once

#include "NodeBase.hpp"

class GainNode : public NodeBase {
public:
    GainNode(double K, NodeBase* input);
    virtual ~GainNode() = default;

    double Get() override;

    void SetGain(double K);
    double GetGain() const;

private:
    double m_K;
    NodeBase* m_input;
};
