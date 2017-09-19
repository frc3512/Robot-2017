/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2017 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "XboxController.h"

#include <HAL/HAL.h>

using namespace frc;

/**
 * Construct an instance of an Xbox controller.
 *
 * The controller index is the USB port on the Driver Station.
 *
 * @param port The port on the Driver Station that the controller is plugged
 *             into (0-5).
 */
XboxController::XboxController(int port) : GenericHID(port) {
  // HAL_Report(HALUsageReporting::kResourceType_XboxController, port);
  HAL_Report(HALUsageReporting::kResourceType_Joystick, port);
}

/**
 * Get the X axis value of the controller.
 *
 * @param hand Side of controller whose value should be returned.
 */
double XboxController::GetX(JoystickHand hand) const {
  if (hand == kLeftHand) {
    return GetRawAxis(0);
  } else {
    return GetRawAxis(4);
  }
}

/**
 * Get the Y axis value of the controller.
 *
 * @param hand Side of controller whose value should be returned.
 */
double XboxController::GetY(JoystickHand hand) const {
  if (hand == kLeftHand) {
    return GetRawAxis(1);
  } else {
    return GetRawAxis(5);
  }
}

/**
 * Get the trigger axis value of the controller.
 *
 * @param hand Side of controller whose value should be returned.
 */
double XboxController::GetTriggerAxis(JoystickHand hand) const {
  if (hand == kLeftHand) {
    return GetRawAxis(2);
  } else {
    return GetRawAxis(3);
  }
}

/**
 * Read the value of the bumper button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 */
bool XboxController::GetBumper(JoystickHand hand) const {
  if (hand == kLeftHand) {
    return GetButton(Button::kBumperLeft);
  } else {
    return GetButton(Button::kBumperRight);
  }
}

/**
 * Read the value of the stick button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool XboxController::GetStickButton(JoystickHand hand) const {
  if (hand == kLeftHand) {
    return GetButton(Button::kStickLeft);
  } else {
    return GetButton(Button::kStickRight);
  }
}

/**
 * Read the value of the A button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool XboxController::GetAButton() const { return GetButton(Button::kA); }

/**
 * Read the value of the B button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool XboxController::GetBButton() const { return GetButton(Button::kB); }

/**
 * Read the value of the X button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool XboxController::GetXButton() const { return GetButton(Button::kX); }

/**
 * Read the value of the Y button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool XboxController::GetYButton() const { return GetButton(Button::kY); }

/**
 * Read the value of the back button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool XboxController::GetBackButton() const { return GetButton(Button::kBack); }

/**
 * Read the value of the start button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool XboxController::GetStartButton() const {
  return GetButton(Button::kStart);
}

/**
 * Get buttons based on an enumerated type.
 *
 * @param button The button enum value.
 * @return The state of the button.
 */
bool XboxController::GetButton(Button button) const {
  return GetRawButton(static_cast<int>(button));
}

/**
 * Whether the button was pressed since the last check.
 *
 * @param button The button enum value.
 * @return Whether the button was pressed since the last check.
 */
bool XboxController::GetButtonPressed(Button button) {
  return GetRawButtonPressed(static_cast<int>(button));
}

/**
 * Whether the button was released since the last check.
 *
 * @param button The button enum value.
 * @return Whether the button was released since the last check.
 */
bool XboxController::GetButtonReleased(Button button) {
  return GetRawButtonReleased(static_cast<int>(button));
}
