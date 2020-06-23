// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <functional>

#include "frc/ctrlsys/INode.h"

namespace frc {

/**
 * Converts a callable into a control system node whose output is the callable's
 * return value.
 */
class FuncNode : public INode {
public:
    FuncNode(std::function<double()> func);  // NOLINT

    double GetOutput() override;

private:
    std::function<double()> m_func;
};

}  // namespace frc
