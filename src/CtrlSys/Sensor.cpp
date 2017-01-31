// Copyright (c) FRC Team 3512, Spartatroniks 2017. All Rights Reserved.

#include "Sensor.hpp"

Sensor::Sensor(frc::PIDSource* source) { m_source = source; }

/**
 * Returns value from sensor
 */
double Sensor::Get() { return m_source->PIDGet(); }
