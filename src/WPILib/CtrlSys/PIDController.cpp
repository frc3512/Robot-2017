/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "CtrlSys/PIDController.h"

#include <cmath>
#include <string>

static const std::string kP = "p";
static const std::string kI = "i";
static const std::string kD = "d";
static const std::string kF = "f";
static const std::string kSetpoint = "setpoint";
static const std::string kEnabled = "enabled";

/**
 * Allocate a PID object with the given constants for P, I, D.
 *
 * @param Kp the proportional coefficient
 * @param Ki the integral coefficient
 * @param Kd the derivative coefficient
 * @param input the node that is used to get values
 * @param output The PIDOutput object that is set to the output value
 * @param period the loop time for doing calculations. This particularly
 *               effects calculations of the integral and differental terms.
 */
PIDController::PIDController(double Kp, double Ki, double Kd, INode& input,
                             PIDOutput& output, double period)
    : m_sum(m_refInput, true, input, false),
      m_pid(Kp, Ki, Kd, m_sum, period),
      m_output(m_pid, output, period) {
  static int instances = 0;
  instances++;
  HAL_Report(HALUsageReporting::kResourceType_PIDController, instances);
}

/**
 * Allocate a PID object with the given constants for P, I, D.
 *
 * @param Kp the proportional coefficient
 * @param Ki the integral coefficient
 * @param Kd the derivative coefficient
 * @param Kff the feedforward coefficient
 * @param input the node that is used to get values
 * @param output The PIDOutput object that is set to the output value
 * @param period the loop time for doing calculations. This particularly
 *               effects calculations of the integral and differental terms.
 */
PIDController::PIDController(double Kp, double Ki, double Kd, double Kff,
                             INode& input, PIDOutput& output, double period)
    : m_sum(m_refInput, true, input, false),
      m_pid(Kp, Ki, Kd, m_feedforward, m_sum, period),
      m_output(m_pid, output, period) {
  m_feedforward.SetGain(Kff);

  static int instances = 0;
  instances++;
  HAL_Report(HALUsageReporting::kResourceType_PIDController, instances);
}

PIDController::~PIDController() { RemoveListeners(); }

/**
 * Set the PID Controller gain parameters.
 *
 * Set the proportional, integral, and differential coefficients.
 *
 * @param Kp Proportional coefficient
 * @param Ki Integral coefficient
 * @param Kd Differential coefficient
 */
void PIDController::SetPID(double Kp, double Ki, double Kd) {
  m_pid.SetPID(Kp, Ki, Kd);
}

/**
 * Set the PID Controller gain parameters.
 *
 * Set the proportional, integral, and differential coefficients.
 *
 * @param Kp Proportional coefficient
 * @param Ki Integral coefficient
 * @param Kd Differential coefficient
 * @param Kff Feedforward coefficient
 */
void PIDController::SetPID(double Kp, double Ki, double Kd, double Kff) {
  m_pid.SetPID(Kp, Ki, Kd);
  m_feedforward.SetGain(Kff);
}

/**
 * Get the Proportional coefficient.
 *
 * @return proportional coefficient
 */
double PIDController::GetP() const { return m_pid.GetP(); }

/**
 * Get the Integral coefficient.
 *
 * @return integral coefficient
 */
double PIDController::GetI() const { return m_pid.GetI(); }

/**
 * Get the Differential coefficient.
 *
 * @return differential coefficient
 */
double PIDController::GetD() const { return m_pid.GetD(); }

/**
 * Get the Feedforward coefficient.
 *
 * @return feedforward coefficient
 */
double PIDController::GetF() const { return m_feedforward.GetGain(); }

/**
 * Set the PID controller to consider the input to be continuous.
 *
 * Rather then using the max and min in as constraints, it considers them to
 * be the same point and automatically calculates the shortest route to
 * the reference (e.g., gyroscope angle).
 *
 * @param continuous true turns on continuous; false turns off continuous
 */
void PIDController::SetContinuous(bool continuous) {
  m_sum.SetContinuous(continuous);
}

/**
 * Sets the maximum and minimum values expected from the input.
 *
 * @param minimumInput the minimum value expected from the input
 * @param maximumInput the maximum value expected from the input
 */
void PIDController::SetInputRange(double minimumInput, double maximumInput) {
  m_sum.SetInputRange(minimumInput, maximumInput);
}

/**
 * Sets the minimum and maximum values to write.
 *
 * @param minimumOutput the minimum value to write to the output
 * @param maximumOutput the maximum value to write to the output
 */
void PIDController::SetOutputRange(double minimumOutput, double maximumOutput) {
  m_output.SetRange(minimumOutput, maximumOutput);
}

/**
 * Set maximum magnitude of error for which integration should occur. Values
 * above this will reset the current total.
 *
 * @param maxErrorMagnitude max value of error for which integration should
 *                          occur
 */
void PIDController::SetIZone(double maxErrorMagnitude) {
  m_pid.SetIZone(maxErrorMagnitude);
}

/**
 * Set the setpoint for the PIDController.
 *
 * @param setpoint the desired setpoint
 */
void PIDController::SetSetpoint(double setpoint) { m_refInput.Set(setpoint); }

/**
 * Returns the current setpoint of the PIDController.
 *
 * @return the current setpoint
 */
double PIDController::GetSetpoint() const { return m_refInput.GetOutput(); }

/**
 * Set the absolute error which is considered tolerable for use with
 * OnTarget.
 *
 * @param tolerance absolute error which is tolerable
 * @param deltaTolerance change in absolute error which is tolerable
 */
void PIDController::SetAbsoluteTolerance(double tolerance,
                                         double deltaTolerance) {
  m_sum.SetTolerance(tolerance, deltaTolerance);
}

/**
 * Returns the current difference of the input from the setpoint.
 *
 * @return the current error
 */
double PIDController::GetError() { return m_sum.GetOutput(); }

/**
 * Return true if the error and change in error is within the range determined
 * by SetTolerance.
 */
bool PIDController::OnTarget() const { return m_sum.InTolerance(); }

/**
 * Begin running the PIDController.
 */
void PIDController::Enable() {
  m_enabled = true;
  m_output.Enable();
}

/**
 * Stop running the PIDController, this sets the output to zero before stopping.
 */
void PIDController::Disable() {
  m_output.Disable();
  m_enabled = false;
}

bool PIDController::IsEnabled() const { return m_enabled; }

void PIDController::Reset() {
  Disable();

  m_pid.Reset();
}

void PIDController::InitTable(std::shared_ptr<nt::NetworkTable> subtable) {
  RemoveListeners();
  if (subtable) {
    m_pEntry = subtable->GetEntry(kP);
    m_pEntry.SetDouble(m_pid.GetP());
    m_iEntry = subtable->GetEntry(kI);
    m_iEntry.SetDouble(m_pid.GetI());
    m_dEntry = subtable->GetEntry(kD);
    m_dEntry.SetDouble(m_pid.GetD());
    m_fEntry = subtable->GetEntry(kF);
    m_fEntry.SetDouble(m_feedforward.GetGain());
    m_setpointEntry = subtable->GetEntry(kSetpoint);
    m_setpointEntry.SetDouble(GetSetpoint());
    m_enabledEntry = subtable->GetEntry(kEnabled);
    m_enabledEntry.SetBoolean(IsEnabled());

    m_pListener = m_pEntry.AddListener(
        [=](const nt::EntryNotification& event) {
          if (!event.value->IsDouble()) return;
          m_pid.SetP(event.value->GetDouble());
        },
        NT_NOTIFY_NEW | NT_NOTIFY_UPDATE);

    m_iListener = m_iEntry.AddListener(
        [=](const nt::EntryNotification& event) {
          if (!event.value->IsDouble()) return;
          m_pid.SetI(event.value->GetDouble());
        },
        NT_NOTIFY_NEW | NT_NOTIFY_UPDATE);

    m_dListener = m_dEntry.AddListener(
        [=](const nt::EntryNotification& event) {
          if (!event.value->IsDouble()) return;
          m_pid.SetD(event.value->GetDouble());
        },
        NT_NOTIFY_NEW | NT_NOTIFY_UPDATE);

    m_fListener = m_fEntry.AddListener(
        [=](const nt::EntryNotification& event) {
          if (!event.value->IsDouble()) return;
          m_feedforward.SetGain(event.value->GetDouble());
        },
        NT_NOTIFY_NEW | NT_NOTIFY_UPDATE);

    m_setpointListener = m_setpointEntry.AddListener(
        [=](const nt::EntryNotification& event) {
          if (!event.value->IsDouble()) return;
          SetSetpoint(event.value->GetDouble());
        },
        NT_NOTIFY_NEW | NT_NOTIFY_UPDATE);

    m_enabledListener = m_enabledEntry.AddListener(
        [=](const nt::EntryNotification& event) {
          if (!event.value->IsBoolean()) return;
          if (event.value->GetBoolean()) {
            Enable();
          } else {
            Disable();
          }
        },
        NT_NOTIFY_NEW | NT_NOTIFY_UPDATE);
  }
}

std::string PIDController::GetSmartDashboardType() const {
  return "PIDController";
}

void PIDController::UpdateTable() {}

void PIDController::StartLiveWindowMode() { Disable(); }

void PIDController::StopLiveWindowMode() {}

void PIDController::RemoveListeners() {
  if (m_pListener != 0) {
    m_pEntry.RemoveListener(m_pListener);
    m_pListener = 0;
  }
  if (m_iListener != 0) {
    m_iEntry.RemoveListener(m_iListener);
    m_iListener = 0;
  }
  if (m_dListener != 0) {
    m_dEntry.RemoveListener(m_dListener);
    m_dListener = 0;
  }
  if (m_fListener != 0) {
    m_fEntry.RemoveListener(m_fListener);
    m_fListener = 0;
  }
  if (m_setpointListener != 0) {
    m_setpointEntry.RemoveListener(m_setpointListener);
    m_setpointListener = 0;
  }
  if (m_enabledListener != 0) {
    m_enabledEntry.RemoveListener(m_enabledListener);
    m_enabledListener = 0;
  }
}