// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <wpi/mutex.h>

#include <units/time.h>

#include "frc/ctrlsys/INode.h"
#include "frc/ctrlsys/NodeBase.h"

namespace frc {

/**
 * Returns the integral of the input node's output.
 */
class DerivativeNode : public NodeBase {
public:
    DerivativeNode(double K, INode& input, units::second_t period = kDefaultPeriod);
    virtual ~DerivativeNode() = default;

    double GetOutput() override;

    void SetGain(double K);
    double GetGain() const;

    void Reset(void);

private:
    double m_gain;
    units::second_t m_period;

    double m_prevInput = 0.0;

    mutable wpi::mutex m_mutex;
};

}  // namespace frc
