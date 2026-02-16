// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/modules/SingleMotorModule.h"
#include <iostream>

SingleMotorModule::SingleMotorModule(int motorID, Config config)
    : motor(motorID, kCANBus) {
    motorConfigs.MotorOutput.Inverted = config.motorInvert;
    motorConfigs.Slot0.kS = config.kS;
    motorConfigs.Slot0.kV = config.kV;
    motorConfigs.Slot0.kA = config.kA;
    motorConfigs.Slot0.kP = config.kP;
    motorConfigs.Slot0.kI = config.kI;
    motorConfigs.Slot0.kD = config.kD;
    motorConfigs.Voltage.PeakForwardVoltage = config.PeakVoltage;
    motorConfigs.Voltage.PeakReverseVoltage = -config.PeakVoltage;
    motorConfigs.TorqueCurrent.PeakForwardTorqueCurrent      = config.PeakCurrent;
    motorConfigs.TorqueCurrent.PeakReverseTorqueCurrent      = -config.PeakCurrent;
    motorConfigs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = config.RampPeriod;
    motorConfigs.MotionMagic.MotionMagicCruiseVelocity       = config.MotionMagicCruiseVelocity;
    motorConfigs.MotionMagic.MotionMagicAcceleration         = config.MotionMagicAcceleration;
    motorConfigs.MotionMagic.MotionMagicJerk                 = config.MotionMagicJerk;

  /* Retry config apply up to 5 times, report if failure */
  ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i) {
    status = motor.GetConfigurator().Apply(motorConfigs);
    if (status.IsOK()) break;
  }
  if (!status.IsOK()) {
    std::cout << "Could not apply motor configs, error code: " << status.GetName() << std::endl;
  }

}