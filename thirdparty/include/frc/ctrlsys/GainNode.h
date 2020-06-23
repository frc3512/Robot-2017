// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <mutex>

#include "frc/ctrlsys/INode.h"
#include "frc/ctrlsys/NodeBase.h"

namespace frc {

class GainNode : public NodeBase {
public:
    GainNode(double K, INode& input);
    virtual ~GainNode() = default;

    double GetOutput() override;

    void SetGain(double K);
    double GetGain() const;

private:
    double m_gain;

    mutable std::mutex m_mutex;
};

}  // namespace frc
