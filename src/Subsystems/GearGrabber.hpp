// Copyright (c) FRC Team 3512, Spartatroniks 2016-2017. All Rights Reserved.

#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include <Compressor.h>
#include <DigitalInput.h>
#include <Joystick.h>
#include <Solenoid.h>
#include <Timer.h>

#include "../Constants.hpp"
#include "../DigitalInputHandler.hpp"
#include "SubsystemBase.hpp"

/* Lowers, grabs, and raises,
 * and then opens with a separate function
 */
class GearGrabber : public SubsystemBase {
public:
    void PickUp();
    void Open();

    void ResetEncoders() override;

private:
    Solenoid m_closingSolenoid{0};   // TODO: find right port
    Solenoid m_loweringSolenoid{1};  // TODO
    Compressor m_robotCompressor;
    frc::Joystick m_gearStick{k_grabberStickPort};

    Timer m_grabberTimer;
};
