// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include "frc/ctrlsys/INode.h"

namespace frc {

/**
 * Provides a common base class for nodes with one input.
 */
class NodeBase : public INode {
public:
    explicit NodeBase(INode& input);
    virtual ~NodeBase() = default;

    INode* GetInputNode() override;
    double GetOutput() override;

private:
    INode& m_input;
};

}  // namespace frc
