// Copyright (c) FRC Team 3512, Spartatroniks 2016-2017. All Rights Reserved.

#pragma once

#include <memory>
#include <string>
#include <utility>
#include <CANTalon.h>
#include <Compressor.h>
#include <DigitalInput.h>
#include <Joystick.h>
#include <Solenoid.h>

#include "../Constants.hpp"
#include "../Events/TimerEventGenerator.hpp"
#include "../Events/JoystickEventGenerator.hpp"
#include "../DigitalInputHandler.hpp"
#include "../SM/StateMachine.hpp"
#include "SubsystemBase.hpp"
#include "GearBox.hpp"

/* Provides a simple state machine for picking up a gear with minimal human input */
class GearPickup : public SubsystemBase {
public:
	GearPickup();

    void Pickup();

    void ReloadPID();
    void ResetEncoders();

private:
    Solenoid pickupSolenoid{0};
    Compressor robotCompressor;
    CANTalon gearMotor{15};
    frc::Joystick m_gearStick{2};

    JoystickEventGenerator m_joystickEvent;
    TimerEventGenerator m_timerEvent{"SolenoidTimeOut", 0.5};

	StateMachine gearPickup{"GearPickup"};

};
