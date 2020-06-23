// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <units/units.h>

namespace frc {

class Output;

/**
 * Interface for control system diagram node.
 *
 * Common interface for control system diagram nodes. Nodes consist of some
 * operation upon an input such as integration, differentiation, multiplying by
 * a constant, or summing multiple inputs together.
 *
 * Subclasses should take at least one input node in their constructor to be
 * used in GetOutput().
 */
class INode {
public:
    virtual ~INode() = default;

    virtual INode* GetInputNode();

    virtual void SetCallback(Output& output);

    /**
     * Performs an operation on the input node's output and returns the result.
     */
    virtual double GetOutput() = 0;

    /**
     * The default period used in control loops in seconds.
     */
    static constexpr units::second_t kDefaultPeriod = 0.05_s;
};

}  // namespace frc
