// Copyright (c) 2017 FRC Team 3512. All Rights Reserved.

#pragma once

#include <memory>

#include "NodeBase.hpp"

/**
 * A node for reference inputs (e.g., setpoints).
 */
class RefInput : public NodeBase {
public:
    explicit RefInput(double reference = 0.0);
    virtual ~RefInput() = default;

    double Get() override;
    double Get() const;

    void Set(double reference);

private:
    double m_reference;
};
