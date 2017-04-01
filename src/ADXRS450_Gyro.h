// Copyright (c) FRC Team 3512, Spartatroniks 2015-2017. All Rights Reserved.

#pragma once

#include <stdint.h>

#include <memory>

#include "GyroBase.h"
#include "HAL/cpp/priority_mutex.h"
#include "Notifier.h"
#include "SPI.h"

namespace frc {

/**
 * Use a rate gyro to return the robots heading relative to a starting position.
 * The Gyro class tracks the robots heading based on the starting position. As
 * the robot rotates the new heading is computed by integrating the rate of
 * rotation returned by the sensor. When the class is instantiated, it does a
 * short calibration routine where it samples the gyro while at rest to
 * determine the default offset. This is subtracted from each sample to
 * determine the heading.
 *
 * This class is for the digital ADXRS450 gyro sensor that connects via SPI.
 */
class ADXRS450_Gyro : public GyroBase {
public:
    ADXRS450_Gyro();
    explicit ADXRS450_Gyro(SPI::Port port);
    virtual ~ADXRS450_Gyro() = default;

    double GetAngle() const override;
    double GetRate() const override;
    void Reset() override;
    void Calibrate() override;
    void SetRateBias(double bias);

private:
    SPI m_spi;
    int32_t m_accumCenter = 0;

    uint16_t ReadRegister(int reg);
};

}  // namespace frc
