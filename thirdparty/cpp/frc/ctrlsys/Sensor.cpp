// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#include "frc/ctrlsys/Sensor.h"

using namespace frc;

Sensor::Sensor(PIDSource& source) : m_source(source) {}

/**
 * Returns value from sensor
 */
double Sensor::GetOutput() { return m_source.PIDGet(); }
