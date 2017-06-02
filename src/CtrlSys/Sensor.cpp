// Copyright (c) 2017 FRC Team 3512. All Rights Reserved.

#include "Sensor.hpp"

Sensor::Sensor(frc::PIDSource* source) { m_source = source; }

/**
 * Returns value from sensor
 */
double Sensor::Get() { return m_source->PIDGet(); }
