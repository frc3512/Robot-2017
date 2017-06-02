// Copyright (c) 2017 FRC Team 3512. All Rights Reserved.

#pragma once

#include <PIDSource.h>

#include "NodeBase.hpp"

/**
 * NodeBase adapter for PIDSource subclasses.
 *
 * Wraps a PIDSource object in the NodeBase interface by returning the output of
 * PIDGet() from Get().
 */
class Sensor : public NodeBase {
public:
    explicit Sensor(frc::PIDSource* source);
    virtual ~Sensor() = default;

    double Get() override;

private:
    frc::PIDSource* m_source;
};
