// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include "frc/PIDSource.h"
#include "frc/ctrlsys/INode.h"

namespace frc {

/**
 * INode adapter for PIDSource subclasses.
 *
 * Wraps a PIDSource object in the INode interface by returning the output of
 * PIDGet() from GetOutput().
 */
class Sensor : public INode {
public:
    Sensor(PIDSource& source);  // NOLINT
    virtual ~Sensor() = default;

    double GetOutput() override;

private:
    PIDSource& m_source;
};

}  // namespace frc
