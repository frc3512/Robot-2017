// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include "frc/ctrlsys/MotionProfile.h"

namespace frc {

/**
 * Provides trapezoidal velocity control.
 *
 * Constant acceleration until target (max) velocity is reached, sets
 * acceleration to zero for a calculated time, then decelerates at a constant
 * acceleration with a slope equal to the negative slope of the initial
 * acceleration.
 */
class TrapezoidProfile : public MotionProfile {
public:
    TrapezoidProfile(double maxV, double timeToMaxV);
    virtual ~TrapezoidProfile() = default;

    void SetGoal(double goal, double currentSource = 0.0) override;

    void SetMaxVelocity(double velocity);
    double GetMaxVelocity() const;
    void SetTimeToMaxV(double timeToMaxV);

protected:
    State UpdateSetpoint(double currentTime) override;

private:
    double m_acceleration;
    double m_velocity;
    double m_profileMaxVelocity;
    double m_timeFromMaxVelocity;
    double m_timeToMaxVelocity;
};

}  // namespace frc
