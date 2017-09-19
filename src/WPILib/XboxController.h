/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2017 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <support/deprecated.h>

#include "ErrorBase.h"
#include "GenericHID.h"

namespace frc {

/**
 * Handle input from Xbox 360 or Xbox One controllers connected to the Driver
 * Station.
 *
 * This class handles Xbox input that comes from the Driver Station. Each time a
 * value is requested the most recent value is returend. There is a single class
 * instance for each controller and the mapping of ports to hardware buttons
 * depends on the code in the Driver Station.
 */
class XboxController : public GenericHID {
 public:
  enum class Button {
    kBumperLeft = 5,
    kBumperRight = 6,
    kStickLeft = 9,
    kStickRight = 10,
    kA = 1,
    kB = 2,
    kX = 3,
    kY = 4,
    kBack = 7,
    kStart = 8
  };

  explicit XboxController(int port);
  virtual ~XboxController() = default;

  XboxController(const XboxController&) = delete;
  XboxController& operator=(const XboxController&) = delete;

  double GetX(JoystickHand hand) const override;
  double GetY(JoystickHand hand) const override;
  double GetTriggerAxis(JoystickHand hand) const;

  WPI_DEPRECATED("Use GetButton() instead.")
  bool GetBumper(JoystickHand hand) const;

  WPI_DEPRECATED("Use GetButton() instead.")
  bool GetStickButton(JoystickHand hand) const;

  WPI_DEPRECATED("Use GetButton() instead.")
  bool GetAButton() const;

  WPI_DEPRECATED("Use GetButton() instead.")
  bool GetBButton() const;

  WPI_DEPRECATED("Use GetButton() instead.")
  bool GetXButton() const;

  WPI_DEPRECATED("Use GetButton() instead.")
  bool GetYButton() const;

  WPI_DEPRECATED("Use GetButton() instead.")
  bool GetBackButton() const;

  WPI_DEPRECATED("Use GetButton() instead.")
  bool GetStartButton() const;

  bool GetButton(Button button) const;
  bool GetButtonPressed(Button button);
  bool GetButtonReleased(Button button);
};

}  // namespace frc
