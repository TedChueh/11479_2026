// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>

#include "subsystems/modules/DualMotorModule.h"

DualMotorModule::DualMotorModule(int motorRightID, int motorLeftID, Config config)
    : motorRight(motorRightID, kCANBus), motorLeft(motorLeftID, kCANBus) {

    motorRightConfigs.MotorOutput.Inverted = config.motorRightInvert;
    motorRightConfigs.Slot0.kS = config.kS;
    motorRightConfigs.Slot0.kV = config.kV;
    motorRightConfigs.Slot0.kP = config.kP;
    motorRightConfigs.Slot0.kI = config.kI;
    motorRightConfigs.Slot0.kD = config.kD;
    motorRightConfigs.Voltage.PeakForwardVoltage = config.PeakVoltage;
    motorRightConfigs.Voltage.PeakReverseVoltage = -config.PeakVoltage;
    motorRightConfigs.TorqueCurrent.PeakForwardTorqueCurrent = config.PeakCurrent;
    motorRightConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -config.PeakCurrent;
    motorRightConfigs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = config.RampPeriod;
    motorRightConfigs.MotionMagic.MotionMagicCruiseVelocity       = config.MotionMagicCruiseVelocity;
    motorRightConfigs.MotionMagic.MotionMagicAcceleration         = config.MotionMagicAcceleration;
    motorRightConfigs.MotionMagic.MotionMagicJerk                 = config.MotionMagicJerk;

    motorLeftConfigs.MotorOutput.Inverted = config.motorLeftInvert;
    motorLeftConfigs.Slot0.kS = config.kS;
    motorLeftConfigs.Slot0.kV = config.kV;
    motorLeftConfigs.Slot0.kA = config.kA;
    motorLeftConfigs.Slot0.kP = config.kP;
    motorLeftConfigs.Slot0.kI = config.kI;
    motorLeftConfigs.Slot0.kD = config.kD;
    motorLeftConfigs.Voltage.PeakForwardVoltage = config.PeakVoltage;
    motorLeftConfigs.Voltage.PeakReverseVoltage = -config.PeakVoltage;
    motorLeftConfigs.TorqueCurrent.PeakForwardTorqueCurrent = config.PeakCurrent;
    motorLeftConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -config.PeakCurrent;
    motorLeftConfigs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = config.RampPeriod;
    motorLeftConfigs.MotionMagic.MotionMagicCruiseVelocity       = config.MotionMagicCruiseVelocity;
    motorLeftConfigs.MotionMagic.MotionMagicAcceleration         = config.MotionMagicAcceleration;
    motorLeftConfigs.MotionMagic.MotionMagicJerk                 = config.MotionMagicJerk;

  /* Retry config apply up to 5 times, report if failure */
  ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i) {
    status = motorRight.GetConfigurator().Apply(motorRightConfigs);
    if (status.IsOK()) break;
  }
  if (!status.IsOK()) {
    std::cout << "Could not apply right motor configs, error code: " << status.GetName() << std::endl;
  }

  for (int i = 0; i < 5; ++i) {
    status = motorLeft.GetConfigurator().Apply(motorLeftConfigs);
    if (status.IsOK()) break;
  }
  if (!status.IsOK()) {
    std::cout << "Could not apply left motor configs, error code: " << status.GetName() << std::endl;
  }

}