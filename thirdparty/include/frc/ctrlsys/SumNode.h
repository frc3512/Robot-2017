// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <limits>
#include <memory>
#include <wpi/mutex.h>
#include <utility>
#include <vector>

#include "frc/ctrlsys/INode.h"

namespace frc {

/**
 * Takes an arbitrary list of input nodes and outputs the sum of their
 * outputs.
 */
class SumNode : public INode {
public:
    template <class... INodes>
    SumNode(INode& input, bool positive, INodes&&... inputs);

    SumNode(INode& input, bool positive);

    double GetOutput() override;

    void SetContinuous(bool continuous = true);
    void SetInputRange(double minimumInput, double maximumInput);

    void SetTolerance(double tolerance, double deltaTolerance);
    bool InTolerance() const;

private:
    /**
     * First argument is input node.
     * Second argument is whether to add or subtract its output.
     */
    std::vector<std::pair<INode&, bool>> m_inputs;

    double m_currentResult = 0.0;
    double m_lastResult = 0.0;

    bool m_continuous = false;
    double m_inputRange = 0.0;

    double m_tolerance = std::numeric_limits<double>::infinity();
    double m_deltaTolerance = std::numeric_limits<double>::infinity();

    mutable wpi::mutex m_mutex;
};

}  // namespace frc

#include "frc/ctrlsys/SumNode.inc"
