// Copyright (c) FRC Team 3512, Spartatroniks 2017. All Rights Reserved.

#pragma once

#include <atomic>
#include <memory>
#include <thread>

#include <Timer.h>

#include "ProfileNode.hpp"

/**
 * A node for reference inputs (e.g., setpoints).
 */
class TrapezoidProfile : public ProfileNode {
public:
    explicit TrapezoidProfile(double maxV, double timeToMaxV,
                              double period = 0.05);
    virtual ~TrapezoidProfile() = default;

    void SetGoal(double goal, double currentInput) override;

    void SetMaxVelocity(double v);
    double GetMaxVelocity() const;
    void SetTimeToMaxV(double timeToMaxV);

protected:
    double Update(double currentInput) override;

private:
    double m_refDisplacement;
    double m_refVelocity;
    double m_refAcceleration;

    double m_maxAcceleration;
    double m_maxVelocity;
    double m_profileMaxVelocity;
    double m_timeFromMaxVelocity;
    double m_timeToMaxVelocity;
    double m_sign;
};
