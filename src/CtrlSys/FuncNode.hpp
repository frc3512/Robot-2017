// Copyright (c) 2017 FRC Team 3512. All Rights Reserved.

#pragma once

#include <functional>

#include "NodeBase.hpp"

/**
 * Converts a callable into a control system node whose output is the callable's
 * return value.
 */
class FuncNode : public NodeBase {
public:
    FuncNode(std::function<double()> func);  // NOLINT

    double Get() override;

private:
    std::function<double()> m_func;
};
